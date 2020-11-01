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
class animationModifiers():
    """
    class which modifies the animation to add effects like an EE strobe, various
    reference frames etc.
    """
    def __init__(self):
        #strobe vars
        self.tfade = None
        self.freq = None
        self.A0 = None
        self.bodyFrame = None
        self.points = None
        self.strobeActive = False
        
        #add COG ref-frames
        #add x,y,z to the animation as a modifier - see ChIrrTools
        
    def _copyChVector(ChVec):
        return chrono.ChVectorD(ChVec.x,ChVec.y,ChVec.z)
        
    
    def addStrobe(self,system,bodyName,tFade=1.5,freq=10,A0=1):
        """
        add a strobe effect to a body reference frame (usually the EE)
        inputs:
            system - ChSystem being animated
            bodyName - name of the body the strobe will be attached to 
            tfade - time for a dot to fade to zero and be deleted
            freq - number of dots added per second
            A0 - default opacity
        """
        self.bodyFrame = system.SearchBody(bodyName)
        self.tFade = tFade
        self.freq = freq
        self.A0 = A0
        self.points = []
        self.strobeActive = True
        self.tLast = time.time()
        self.trajHistory = None
        self.n = 0
        self.dt = .005
        
        
    def updateStrobe(self,IrrApp):
        self.n += 1
        if self.n == 1:
            pos = self.bodyFrame.GetPos()
            posCopy = chrono.ChVectorD(pos.x,pos.y,pos.z)
            self.points.append(posCopy)
            self.trajHistory = chrono.vector_ChVectorD(self.points)
            
        #t = time.time()
        #if t - self.tLast > (1/self.freq):
           # self.tLast = t
        if self.n % 20 == 0:
            pos = self.bodyFrame.GetPos()
            posCopy = chrono.ChVectorD(pos.x,pos.y,pos.z)
            self.points.append(posCopy)
            if len(self.points) > int(self.tFade * self.freq):
                self.points.pop(0)
            self.trajHistory = chrono.vector_ChVectorD(self.points)
            
            
        #if t % 5 < .01:
        if self.n % 5000 == 0:
            print("interation ",self.n)
            for i in self.points: print(i)
            print("         ")
            #for i in self.points: print(i)
            #print(self.n)
            #print(self.points)
            #print(self.points[0])
            #print(self.points[9])
            pass
        chronoirr.ChIrrTools_drawPolyline(IrrApp.GetVideoDriver(),self.trajHistory)
        #chronoirr.ChIrrTools_drawPolyline(IrrApp.GetVideoDriver(),chrono.vector_ChVectorD([chrono.ChVectorD(0,0,0),chrono.ChVectorD(1,1,1)]))
        #chronoirr.ChIrrTools_drawCircle(IrrApp.GetVideoDriver(),.5)
        
        
        """ it's a pointer... you dummy, you have to make a copy!!"""
        
        
    def addCOGrfs(self,IrrApp):
        pass
    
    def draw(self,IrrApp):
        """
        draw all the animation modifiers
        """
        if self.strobeActive:   self.updateStrobe(IrrApp)
    
    
    
    """
    what's the plan?  quiery the system object for it's position (given that we have its name)
    then we add that position to a queue of a fixed length. (popping the oldest one, add ChVector objects) the queue
    is implemented as a python list, and each update call we mint a vector of vectors for the draw polyLine fxn. 
    we should also see what the deal is with the color function, to see if we can adjust the alpha and maybe the color too?
    and the thickness
    """
    
    #chronoirr.ChIrrTools_drawAllCOGs(system,myapplication.GetVideoDriver())
    #chronoirr.ChIrrTools_drawPolyline(myapplication.GetVideoDriver(),chrono.ChVectorD(0,0,-0.2))
    
    #drawPolyline (irr::video::IVideoDriver *driver, std::vector< ChVector<> > &mpoints, irr::video::SColor mcol=irr::video::SColor(255, 0, 0, 0), bool use_Zbuffer=false)
 	#Easy-to-use function to draw a polyline in 3D space, given the array of points as a std::vector.
    #chrono.vector_ChVectorD([chrono.ChVectorD(1,1,1),chrono.ChVectorD(1,1,1)])
     
    #chrono.vector_ChVectorD
    


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
        


# -*- coding: utf-8 -*-
"""
contains functions for visualizing the results of a simulation in realtime, 
or graphing different variables over time. 
"""
#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr

import numpy as np
import matplotlib.pyplot as plt





#---------------------- function definitions ----------------------------------

def animateSystem(system):
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
    
    
    # -----------------------------------------------------------------------------
    #                             Run the simulation
    # -----------------------------------------------------------------------------
    
    
    myapplication.SetTimestep(0.005)
    
    
    while(myapplication.GetDevice().run()):
        myapplication.BeginScene()
        myapplication.DrawAll()
        myapplication.DoStep()
        myapplication.EndScene()
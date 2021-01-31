# -*- coding: utf-8 -*-
"""
utility functions that get used throughout the project
"""
import numpy as np

def Rx(θ):
    return np.array([[1, 0 ,0],
                     [0,np.cos(θ), np.sin(θ)],
                     [0,np.sin(θ), np.cos(θ)]])


    

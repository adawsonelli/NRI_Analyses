"""
library of functions for constructing chrono models
"""

#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
from models import 

# Change this path to asset path, if running from other working dir. 
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chronoDataDir = "D:/programFiles/Miniconda3/pkgs/pychrono-5.0.0-py37_9/Library/data/"
chrono.SetChronoDataPath(chronoDataDir)
assetsPath = "C:/Users/adaws/Documents/gitRepos/NRI_Analyses/chrono/assets/"


#------------------- public facing functions ----------------------------------

def genMinJerkTrajectory(Xs,Ys,tf,npts):
    """
    pass
    """
    pass


def CubicPoly(Xs,Ys,Vxs,Vys,ts,nSamps):
    """
    create a cubic polynomial trajectory of the end effector from a series of 
    locations (end points and via points), velocities, and times - specifiying 
    the number of samples.
    inputs:
        Xs -  the x - locations of the via points [list or np.array]
        Ys -  the y - locations of the via points [list or np.array]
        Vxs - the x velocity at each point - None if it doesn't matter. [list or np.array]
        Vys - the y velocity at each point - None if it doesn't matter. [list or np.array]
        ts - the time samples - not strictly uniformly sampled
        nSamps - the number of samples in the output to uniformly sample between [0 - tf]
    """
    pass

def CubicPolyJoints(Xs,Ys,Vxs,Vys,ts,nSamps):
    """
    from a specification of end-effector cubic Poly parameters, determine trajectories
    for each of the 2 driving joints, θ1l and θ1r
    inputs:
        same as Cubic Poly
    outputs:
        θ1l - time trajectory of  and θ1r
    """
    pass





#
#function [t y yd ydd] = QuinticPolynomial(t0, tf, y0, yf, yd0, ydf, ydd0, yddf, n)
#%QUINTICPOLYNOMIAL creates a quinitic spline fit given the initial time and
#%kinematic parameters. 
#%inputs: 
#    % t0   - initial time
#    % tf   - final time
#    % y0   - inital position
#    % yf   - final position
#    % yd0  - initial velocity
#    % ydf  - final velocity
#    % ydd0 - initial acceleration
#    % yddf - final acceleration
#    % n    - number of samples
#%outputs: 
#    % t  - [1xn] time vector (evenly spaced row vector)
#    % y  - [1xn] position vector
#    % yd - [1xn] velocity vector
#    %ydd - [1xn] acceleration vector
#
#%solve for co-efficients a0 - a5 using matrix inversion
#% Ca = q   ->  a = C/q
#    
#%coefficient matrix  
#C = [1    t0   t0^2   t0^3    t0^4     t0^5
#     0    1    2*t0   3*t0^2  4*t0^3   5*t0^4 
#     0    0    2      6*t0    12*t0^2  20*t0^3
#     1    tf   tf^2   tf^3    tf^4     tf^5
#     0    1    2*tf   3*tf^2  4*tf^3   5*tf^4
#     0    0    2      6*tf    12*tf^2  20*tf^3];
#
#y = [y0 yd0 ydd0 yf ydf yddf]'; %specified positions/velocities
#
#a = C\y; %calculated quintic polynomial coefficients
#
#%----use the quintic spline co-efficients to create time vectors---------
#t = linspace(0,tf,n);
#
#%extract co-efficients
#a0 = a(1); a1 = a(2); a2 = a(3); a3 = a(4); a4 = a(5); a5 = a(6);
#
#%solve for position, velocity, acceleration vectors
#y   =   a0 +   a1*t +    a2*t.^2 +    a3*t.^3 +   a4*t.^4 + a5*t.^5;
#yd  =   a1 + 2*a2*t +  3*a3*t.^2 +  4*a4*t.^3 + 5*a5*t.^4;
#ydd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
#
#
#end

    

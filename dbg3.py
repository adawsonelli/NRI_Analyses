#
#
#
##%% 
#
#import numpy as np
#from numpy import linalg as LA
#from shapely.geometry import Polygon
#
#def Jac(θ1,θ2,l1,l2):
#    """return the jacobian of an RR robot given joint angles"""
#     #define sin and cosine constants for this evaluation
#    s1  = np.sin(θ1)      ; c1  = np.cos(θ1)
#    s2  = np.sin(θ2)      ; c2  = np.cos(θ2)
#    s12 = np.sin(θ1 + θ2) ; c12 = np.cos(θ1 + θ2)
#    
#    #calculate the jacobian 
#    jac = np.array([[-l1*s1 - l2*s12   ,l1*c1 +l2*c12   ], 
#                    [-l2*s12           , l2*c12         ]])
#    return jac
#    
#
#def τs2ForcePolygon(τs,Θ):
#    cs = (LA.inv(Θ) @  τs.corners()).T
#    fcs = [(cs[i,0], cs[i,1]) for i in range(cs.shape[0])]
#    return Polygon(fcs)
#    
#
#class TauSpace():
#    """define a torque space"""
#    def __init__(self,t1max,t1min,t2max,t2min):
#        self.t1max = t1max
#        self.t1min = t1min
#        self.t2max = t2max
#        self.t2min = t2min
#    def corners(self):
#        """return the corners of the torque space polygon"""
#        cs = np.array([[self.t1max,self.t1min,self.t1min,self.t1max],
#                       [self.t2max,self.t2max,self.t2min,self.t2min]])
#        return cs
#
#def test_PolygonLT():
#    #define a torque space
#    τs = TauSpace(-1,1,-1,1)
#    count = 0
#
#    #let's test the area of the polygon for a variety of jacobians
#    n =  100
#    θs = np.linspace(0,2*np.pi,n)
#    for i,θ1 in enumerate(θs):
#        for j,θ2 in enumerate(θs):
#            Θ = Jac(θ1,θ2,1,1)
#            if LA.cond(Θ) < 100:
#                poly = τs2ForcePolygon(τs,Θ)
#
#                #is the calculated area wrong? 
#                ϵ = .001;
#                if ( abs(poly.area - abs(np.linalg.det(LA.inv(Θ)) * 4))  > ϵ):
#                    count += 1
#    np.testing.assert_equal(count, 0)
#    
#    
#test_PolygonLT()
#                
#

import numpy as np


def noNans2(*args):
    for arg in args:
        #item is iterable and therefore needs to be delt with recursively
        if hasattr(arg, '__iter__'):
            for a  in arg:
                nn = noNans2(a)
                if nn == False:
                    return False
                
        #item is not iterable, and can therefore be considered individually
        else:
            if np.isnan(arg):
                return False
    return True
    

noNans2(1,2,3,(1,2,4,np.nan))




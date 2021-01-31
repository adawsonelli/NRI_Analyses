## %%
##------------- imports --------------
#import plotly.graph_objs as go
#import numpy as np
#from ipywidgets import interact , Layout , FloatSlider
#from numpy import linalg as LA
#from shapely.geometry import Polygon
#
#
##------------- classes ------------------
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
#    #define polygon more organically? 
#        
#        
#
##------------- globals ------------------
##vars
#H1 = 1  ; H2 = 1  #human limb lengths 
#R1 = 1  ; R2 = 1  #robot link lengths
#
#
#
##--------- helper functions -------------
#def mkCircle(x0,y0,r):
#    t = np.linspace(0,2* np.pi,100)
#    x = r * np.cos(t) + x0
#    y = r * np.sin(t) + y0
#    return x,y
#
#
#def circIntersection(x1,y1,r1,x2,y2,r2):
#    """characterize circle intersection"""
#    dx = x2-x1; dy = y2-y1
#    hyp = (dx**2 + dy**2)**.5
#    if hyp > (r1 + r2):
#        intersect = False
#    else:
#        intersect = True
#    ipt = (x1 + (dx/2), y1 + (dy/2))
#    ang = np.arctan2(dy,dx)
#    return (ipt,ang,intersect)
#
#def withinIntersection(x1,y1,r1,x2,y2,r2,pt):
#    """is the point contained within the intersection?"""
#    def within(x,y,r,pt):
#        dx = pt[0] - x ; dy = pt[1] - y
#        hyp = (dx**2 + dy**2)**.5
#        return r > hyp
#    
#    return (within(x1,y1,r1,pt) and within(x2,y2,r2,pt))
#
#
#def FK(θ1,θ2,l1,l2):
#    """forward kinematics of an RR robot"""
#    s1  = np.sin(θ1)      ; c1  = np.cos(θ1)
#    s2  = np.sin(θ2)      ; c2  = np.cos(θ2)
#    s12 = np.sin(θ1 + θ2) ; c12 = np.cos(θ1 + θ2)
#    
#    #forward kinematics
#    origin = [0,0]
#    l1_tip = [l1*c1 , l1*s1]                                     
#    l2_tip = [l1_tip[0] + l2*c12, l1_tip[1] + l2*s12]
#    return (l1_tip,l2_tip)
# 
#    
#def FKplot(θ1,θ2,l1,l2):
#    """return the x's and y's to plot a robot"""
#    l1_tip,l2_tip = FK(θ1,θ2,l1,l2)
#    xs = np.array([0,l1_tip[0],l2_tip[0]])
#    ys = np.array([0,l1_tip[1],l2_tip[1]])
#    return xs,ys
#  
#    
#def IK(x,y,l1,l2,flipElbow = False):
#    """inverse kinematics of an RR robot"""   
#    θ2 = np.arccos((x**2 + y**2 - l1**2 - l2**2) / (2*l1*l2))
#    β = np.arctan2(y,x)
#    γ =  - np.arcsin((l2*np.sin(θ2)) / (x**2 + y**2)**.5)
#    
#    if not flipElbow:
#        θ1 = β + γ 
#    if flipElbow:
#        θ2 = -θ2
#        θ1 = β - γ 
#        
#    return (θ1,θ2)
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
#
#def Score(τsH,τsR,ΘH,ΘR,mode = "normalize"):
#    """
#    inputs: 
#        τsH - the torque space of the human (type = TauSpace)
#        τsR - the torque space of the robot (type = TauSpace)
#        ΘH - the force jacobian of the human (2x2 np.array)
#        θR - the force jacobian of the robot (2x2 np.array)
#        mode - flag to signal what mode is being used (string)
#    outputs: 
#        score assessing how well the two jacobians overlap (float on 0-1)
#    """
#    
#    #test that the robot or human are not too close to a singularity, if they are
#    #just return zero to avoid dealing with numericial issues. 
#    ϵ = 100    #condition numbers exist on (1- infinity)
#    if (LA.cond(ΘH) > ϵ or LA.cond(ΘR) > ϵ):
#        return 0 
#    
#    #find the force polygons
#    def τs2ForcePolygon(τs,Θ):
#        cs = Θ @ τs.corners()
#        fcs = [(cs[0,i], cs[1,i]) for i in range(cs.shape[1])]
#        return Polygon(fcs)
#    
#    fpolyH = τs2ForcePolygon(τsH,ΘH)
#    fpolyR = τs2ForcePolygon(τsR,ΘR)
#    
#    #determine the score based on area
#    intersection = fpolyH.intersection(fpolyR)
#    if mode == "normalize":
#        return intersection.area / (fpolyH.area + fpolyR.area - intersection.area)
#    elif mode == "contain":
#        return intersection.area / fpolyH.area
#            
#
## def Score1(R_basis,H_basis):
##     """
##     represent H withing the R basis , and determine if H is contained withing R 
##     returns score which saturates on 0-1
##     """
##     try:
##         R_inv = LA.inv(R_basis);
##         H_in_R_basis = r_inv * h_basis
##         return min(1 , LA.norm(H_in_R_basis,np.inf))
##     except:
##         return 0 #we are at a singularity of R_basis
#
#def genEvalBox(x1,y1,r1,x2,y2,r2,nSamps = 10): 
#    """
#    given 2 circles(defined by their coordinates,
#    generate coordinates over which to evaluate
#    input:
#        density - [samples / normalized length]
#    output:
#        x - np.array of x values to sample over
#        y - np.array of y values to sample over
#    """
#    #identify critical points on each circle
#    def pts(x,y,r): #[right,top,left,bottom]
#        return  np.array([[r, 0,-r, 0],
#                          [0, r, 0,-r]]) + np.array([[x],[y]])
#    pts1 = pts(x1,y1,r1) ; pts2 = pts(x2,y2,r2)
#    
#    #find the corners of the box w/ mini-max and maxi-min problem
#    tl = (min(pts1[0,0],pts2[0,0]),min(pts1[1,1],pts2[1,1]))
#    br = (max(pts1[0,2],pts2[0,2]),max(pts1[1,3],pts2[1,3]))
#    dx = tl[0] - br[0]
#    dy = tl[1] - br[1]  
##     x = np.linspace(br[0],tl[0],int(density * dx))
##     y = np.linspace(br[1],tl[1],int(density * dy))
#    x = np.linspace(br[0],tl[0],nSamps)
#    y = np.linspace(br[1],tl[1],nSamps)
#    
#    return x,y
#
#def ScoreIntersectedSpace(x1,y1,r1,x2,y2,r2,FR,mode = "normalized"):
#    """
#    score the area of intersection between the human and the robot
#    and return the parameters of the contour plot
#    input: 
#        circle parameters - for the two workspaces of the robots. 
#        FR - force ratio, how much stronger the robot is than the human.
#        mode - normalized or not?
#    output:
#        X - 1d np.array of length n
#        Y - 1d np.array of length m
#        Z - 2d np.array of size n x m       
#    """
#    #where to sample in x,y coordinates
#    X , Y = genEvalBox(x1,y1,r1,x2,y2,r2,nSamps = 25)
#    Z = np.zeros((X.shape[0],Y.shape[0]))
#    Z = np.full_like(Z, np.nan)
#    
#    #define the torque spaces for the robot and human
#    τsH = TauSpace(1,-1,1,-1);
#    τsR = TauSpace(FR,-FR,FR,-FR);
#    
#    #for each point, evaluate the score
#    for i,x in enumerate(X):
#        for j,y in enumerate(Y):
#            if withinIntersection(x1,y1,r1,x2,y2,r2,(x,y)):
#                #determine the joint angles
#                Hθ1,Hθ2 = IK(x,y,H1,H2)
#                Rθ1,Rθ2 = IK(x - x2,y - y2,R1,R2)
#                
#                #calculate the jacobians
#                ΘH = Jac(Hθ1,Hθ2,H1,H2)
#                ΘR = Jac(Rθ1,Rθ2,R1,R2)
#                
#                #score the jacobian intersections
#                Z[i,j] = Score(τsH,τsR,ΘH,ΘR,mode)
#    
#    return X,Y,Z
#                
#                
#
#    
#    
##-------- plotting routines -------------
##init figure
#X,Y,Z = ScoreIntersectedSpace(0,0,2,1,0,2,1,mode = "normalized")
#_fig = go.Figure(data = go.Contour(z=Z,x=X,y=Y))
#fig = go.FigureWidget(_fig)
#
#
#
##manually configure axes and aspect ratio
#fig.update_xaxes(range=[-6, 6])
#fig.update_yaxes(range=[-6, 6])
#side = 1000
#fig.update_layout(
#    autosize=False,
#    width=side + 100,
#    height=side,
#    legend=dict(x=.025, y=.975))
#
#
##make plot with original circle, and model robot in it.
#(xs,ys) = mkCircle(0,0,H1 + H2);
#fig.add_scatter(x=xs,y=ys,line=dict(dash='dash',color='royalblue'),name="human workspace")
#fig.add_scatter(line=dict(color='royalblue', width=2)             ,name="leg")
#fig.add_scatter(line=dict(dash='dash',color='red')                ,name="robot workspace")
#fig.add_scatter(line=dict(color='red', width=2)                   ,name="robot")
#
#
#line=dict(color='firebrick', width=4)
#
##setup slider objects
#layout=Layout(width='700px', height='20px')
#x1 = FloatSlider(min=-4,max=4,step=0.01,value=1, layout=layout)
#y1 = FloatSlider(min=-4,max=4,step=0.01,value=0, layout=layout)
#FR = FloatSlider(min=1 ,max=5,step=0.1,value=1, layout=layout)
#
#@interact(mode=["none","normalize","inscribed"],x1=x1, y1=y1,FR=FR)
#def update(mode="none",x1=1, y1=0, FR=1):
#    with fig.batch_update():
#        
#        #draw the contour plot of the scoring function
#        if mode == "none":
#            pass
##             #set visability to false
##             xRange,yRange = genEvalBox(0,0,H1 + H2,x1,y1, R1 + R2)
##             fig.data[0]['x'] = xRange
##             fig.data[0]['y'] = yRange
##             fig.data[0]['z'] = np.ones((xRange.shape[0],yRange.shape[0]))
#            
#        if (mode == "normalize" or mode == "inscribed"):
#            xs,ys,zs = ScoreIntersectedSpace(0,0,H1 + H2,x1,y1, R1 + R2,FR,mode)
#            fig.data[0]['x'] = xs
#            fig.data[0]['y'] = ys
#            fig.data[0]['z'] = zs
#            
#    
#        #draw the robot circle
#        xs,ys = mkCircle(x1,y1,R1 + R2)
#        fig.data[3]['x']=xs
#        fig.data[3]['y']=ys
#        
#        #draw the two robots at the center of the workspaces
#        (ipt,ang,intersect) = circIntersection(0,0,H1 + H2,x1,y1, R1 + R2)    
#        if intersect:
#            #human
#            θ1,θ2 = IK(ipt[0],ipt[1],H1,H2,flipElbow = True)
#            xs, ys = FKplot(θ1,θ2,H1,H2)
#            fig.data[2]['x']= xs
#            fig.data[2]['y']= ys
#            
#            #robot
#            θ1,θ2 = IK(ipt[0] - x1,ipt[1] - y1,R1,R2)
#            xs, ys = FKplot(θ1,θ2,R1,R2)
#            fig.data[4]['x']= xs + x1
#            fig.data[4]['y']= ys + y1
#                  
#
##display interactive figure
#fig
#
#
## %%
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
#def testPolygonLT():
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
#            cs = (Θ @  τs.corners()).T
#            fcs = [(cs[i,0], cs[i,1]) for i in range(cs.shape[0])].append(cs[0,:])
#            poly = Polygon(fcs)
#            
#            #is the calculated area wrong? 
#            ϵ = .001;
#            if ( abs(poly.area - np.linalg.det(Θ))  > ϵ):
#                count += 1
#    print(count / n**2)
#    
#testPolygonLT()
#                

##%%
#
#
#class utils:
#    """
#    utility functions shared between different classes
#    """
#    @staticmethod
#    def dist(x1,y1,x2,y2):
#        """
#        calculate the distance between 2 points
#        """
#        return ((x2 - x1)**2 + (y2 - y1)**2)**.5
#    
#    @staticmethod
#    def ROC(θ,upper=None,lower=None):
#        """
#        rollover correction, to make sure the change in angle (Δθ) is accurately
#        represented. the function can equally be used to find the minimal representation
#        of an angle on the range (-π,π)
#        """
#        if upper == None: upper =  np.pi
#        if lower == None: lower = -np.pi
#            
#        while (θ > upper) or (θ <=  lower):
#            if     θ >  upper: θ -= 2*np.pi
#            elif   θ <= lower: θ += 2*np.pi
#        return θ
#    
#    @staticmethod
#    def nanCheck(*args):
#        for arg in args:
#            if np.isnan(arg):
#                return True
#        return False
#    
#    @staticmethod
#    def GD(x0,df,params):
#        """
#        peform gradient decent, given df, a function for calculating the 
#        derivate of the function to be optimized. 
#        """
#        next_x = x0   # We start at x0
#        gamma = 0.01  # Step size multiplier
#        precision = 0.0001  # Desired precision of result
#        max_iters = 10000  # Maximum number of iterations
#        
#        for _ in range(max_iters):
#            current_x = next_x
#            next_x = current_x - gamma * df(current_x,params)
#
#            step = next_x - current_x
#            if abs(step) <= precision:
#                break
#
#        return next_x
#
#
#import numpy as np
##import ipywidgets as widgets
##from ipywidgets import interact , Layout , FloatSlider , Checkbox , Dropdown
##from IPython.display import display
##from numpy import linalg as LA
##from shapely.geometry import Polygon
#
#class Gen5barlinkage:
#    """
#    this general 5 bar linkage class solves the kinematics, inverse kinematics, and 
#    differential kinematics (jacobian) of a 2DOF, 5 bar linkage which is parametrized
#    to be as general a 5 bar linkage as possible. 
#    
#    design parameters: 
#        states that do not change during the course of the inner-loop of the optimization
#        function
#    
#    state variables: 
#        angles that 
#        
#        
#    """
#    def __init__(self):
#        #---continuous design parameters -----------
#        self.x1 = 0
#        self.y1 = 0
#        self.x2 = 1
#        self.y2 = 1
#        
#        self.L0 = 1      #calculated
#        self.L1 = 1      #left link 1
#        self.L2 = 1      #left link 2
#        self.L3 = 1      #left link 3 
#        self.L1r = 1     #right link 1
#        self.L2r = 1     #right link 2
#        
#        self.θ3 = 0       #angle parameter on [0-2π]
#        
#        #------discrete design paramters ---------
#        
#        #effecting the IK
#        self.e1 = 'up'  #[up or down]
#        self.e3 = 'up'
#        
#        #effecting FK
#        self.e2 = 'up'
#        
#        #---------- state variables -----------
#        self.θ1 = None
#        self.θ2 = None
#        self.θ3 = None
#        self.θ1r = None
#        self.θ2r = None
#    
#    
#    def _IK_RRR(self,x,y,θ3):
#        """
#        perform Inverse Kinematics on an RRR robot (constituting the left side the 
#        5bar linkage in the reference position), in terms of x,y,θ3 this is an analytical
#        solution to the IK problem, that assumes θ3 is locked and uses a fictutious link (l_f)
#        approach. 
#        """
#        #define local vars: 
#        L1 = self.L1; L2 = self.L2; L3 = self.L3;
#        
#        #determine the wrist position based on θ3 #keep an eye on this, change to
#        θ3 = utils.ROC(θ3,lower = 0,upper = 2*np.pi)
#        if θ3 >= 0 and θ3 < np.pi:
#            wrist = "down"
#        if θ3 >= np.pi and θ3 > 2*np.pi:
#            wrist = "up"
#        
#        #solve the intermediate values
#        if wrist == "up":     a = θ3 - 180
#        elif wrist == "down": a = 180 - θ3
#        
#        #solve intermediate values
#        Lf = (L2**2 + L3**2 - 2*(L2)*(L3)*np.cos(a))**.5
#        b = np.arcsin(L3*np.sin(a)/Lf)
#           
#        #determine the IK for the RR robot formed by L1 and the fictitious Lf 
#        θ1, θ12f = self._IK_RR(x,y,L1,Lf,self.e1)
#        
#        #determine (θ1,θ2) based on the wrist position
#        if wrist == "up"  : θ2 = θ12f + b - θ1     
#        if wrist == "down": θ2 = θ12f - b - θ1
#            
#        return θ1,θ2,θ3
#    
#
#    
#    def _IK_RR(self,x,y,l1,l2,elbow):
#        """
#        perform Inverse Kinematics on an RR robot, see lect5 p.9-10 
#        """
#        #solve for intermediate values
#        r = utils.dist(0,0,x,y)
#        β = np.arccos((l1**2 + l2**2 - r**2) / (2*l1*l2))
#        γ = np.arccos((r**2 + l1**2 - l2**2) / (2*r*l1))
#        α = np.arctan2(y,x)
#        
#        #handle different elbow states
#        θ1 = α - γ
#        θ2 = np.pi - β
#        
#        if elbow == "up":
#            return θ1,θ2
#        elif elbow == "down":
#            θ1pm = θ1 + 2*γ  
#            θ2pm = -θ2   
#            return θ1pm,θ2pm
#        
#    
#    def _FK_RRR(self,θ1,θ2,θ3,l1,l2,l3):
#        """
#        forward kinematics for the an RRR robot
#        """
#        s1  = np.sin(θ1)      ; c1  = np.cos(θ1)
#        s12 = np.sin(θ1 + θ2) ; c12 = np.cos(θ1 + θ2)
#        s123 = np.sin(θ1 + θ2 + θ3) ; c123 = np.cos(θ1 + θ2 + θ3)
#    
#        #forward kinematics
#        origin = [0,0]
#        l1_tip = [l1*c1 , l1*s1]                                     
#        l2_tip = [l1_tip[0] + l2*c12, l1_tip[1] + l2*s12]
#        l3_tip = [l2_tip[0] + l3*c123, l2_tip[1] + l3*s123]
#        
#        #reorganize into vectors
#        x = [origin[0],l1_tip[0],l2_tip[0],l3_tip[0]]
#        y = [origin[1],l1_tip[1],l2_tip[1],l3_tip[1]]
#        
#        return x,y
#    
#    def _FK_RR(self,θ1,θ2,l1,l2):
#        """
#        forward kinematics for RR robot
#        """
#        x,y = self._FK_RRR(θ1,θ2,0,l1,l2,0)
#        return x[:-1],y[:-1]
#    
#    def IK5bl(self,x,y):
#        """
#        calculate the inverse kinematics for the entire 5-bar-linkage robot. 
#        """
#        #see if the left RRR robot can reach the target point with the end effector
#        θ1,θ2,θ3 = self._IK_RRR(x - self.x1,y - self.y1,self.θ3)
##         if utils.nanCheck(θ1,θ2):
##             return np.nan
#        
#        #calculate the position of joint e2
#        xs,ys = self._FK_RR(θ1,θ2,self.L1,self.L2)                      #local to RRR link
#        xe2 = xs[1] + self.x1; ye2 = ys[1] + self.y1                    #global frame
#        
#        #calculate the IK for tip of the right RR robot to the point E2
#        θ1r,θ2r = self._IK_RR(x - self.x2,y - self.y2,self.L1r,self.L2r,self.e3)
##         if utils.nanCheck(θ1r,θ2r):
##             return np.nan
#        
#        #if we made it this far, we have sucessfully performed IK, update state and return
#        #maybe we don't use the state? don't know at this point, could be useful for caching / s
#        #saving on computation? 
#        self.θ1,self.θ2,self.θ3,self.θ1r,self.θ2r  =  θ1,θ2,θ3,θ1r,θ2r 
#        return θ1,θ2,θ3,θ1r,θ2r 
#    
#    
#    def FKplot5bl(self,θ1,θ2,θ3,θ1r,θ2r):
#        """
#        given the joint angles, what are the points to plot to view the robot?
#        return the points as 3 objects so they can be plotted in different styles
#        """
#        xs,ys = self._FK_RRR(θ1,θ2,θ3,self.L1,self.L2,self.L3)        #local to RRR link
#        xs = self.x1 + np.array(xs) ; ys = self.y1 + np.array(ys)     #global frame
#        
#        left        = (xs[0:3],ys[0:3])
#        endEffector = (xs[2: ],ys[2: ])
#        
#        xs,ys = self._FK_RRR(θ1r,θ2r,self.L1r,self.L2r)               #local to RRR link
#        xs = self.x2 + np.array(xs) ; ys = self.y2 + np.array(ys)     #global frame
#        
#        right = (xs,ys)
#        
#        return left,endEffector,right
#        
#        
#        
#    def Jac(self,x,y):
#        """
#        calculate the jacobian matrix at position x,y
#        """
#        θ1,θ2,θ3,θ1r,θ2r  =  IK5bl(self,x,y)
#
#
#        
#        
#R = Gen5barlinkage()
#a = R._IK_RRR(2,1,.5)
#
#

#%%

import numpy as np
from numpy import linalg as LA
from shapely.geometry import Polygon
from scipy.spatial import ConvexHull
from scipy.optimize import minimize

    
##design decision - should I treat Human more functionally - so basically as a static class, but without state, and more of a container for functions, or should I rely heavily 
#on state caching, which means I have to guarentee order of function calls inorder for things to be up to date. this is the fundemental tradeoff of functional vs stateful approaches. 
# other design decisions, would sampling happen in cartesian, or in joint space for the optimization? - at least some of that sampling should live in another class that handles the
#optimization between the human and the gen5bl




#----------------------------------------------- utils ---------------------------------------------------------------

class utils:
    """
    utility functions shared between different classes
    """
    @staticmethod
    def dist(x1,y1,x2,y2):
        """
        calculate the distance between 2 points
        """
        return ((x2 - x1)**2 + (y2 - y1)**2)**.5
    
    @staticmethod
    def mkCircle(x0,y0,r,n = 100):
        t = np.linspace(0,2* np.pi,n)
        x = r * np.cos(t) + x0
        y = r * np.sin(t) + y0
        return x,y

    @staticmethod
    def circIntersection(x1,y1,r1,x2,y2,r2):
        """characterize circle intersection"""
        dx = x2-x1; dy = y2-y1
        hyp = (dx**2 + dy**2)**.5
        if hyp > (r1 + r2):
            intersect = False
        else:
            intersect = True
        ipt = (x1 + (dx/2), y1 + (dy/2))
        ang = np.arctan2(dy,dx)
        return (ipt,ang,intersect)

    @staticmethod
    def withinIntersection(x1,y1,r1,x2,y2,r2,pt):
        """is the point contained within the intersection or two circles?"""
        def within(x,y,r,pt):
            dx = pt[0] - x ; dy = pt[1] - y
            hyp = (dx**2 + dy**2)**.5
            return r > hyp

        return (within(x1,y1,r1,pt) and within(x2,y2,r2,pt))
    
    @staticmethod
    def ROC(θ,upper=np.pi,lower=-np.pi):
        """
        rollover correction, to make sure the change in angle (Δθ) is accurately
        represented. the function can equally be used to find the minimal representation
        of an angle on the range (-π,π)
        """
        while (θ > upper) or (θ <  lower):
            if     θ >  upper: θ -= 2*np.pi
            elif   θ <= lower: θ += 2*np.pi
        return θ
    
    
    #notes on these angle helper functions: they all have the restriction of only working for the 
    #short way arround. if we want them to work for the long way arround, they need to be rethought
    # in the future. 
    @staticmethod
    def clampAng(ang,lower,upper):
        """
        clamp angle ang between the lower and upper angles. the implementation assumes that
        the distance between lower and upper is < π rad, and that the lower angle is "to the right"
        of the upper angle, in the context of the two angles being closer than π
        
        #note that this function doesn't operate sucessfully if lower and upper are switched, if 
         if the desired bounds between them are > np.pi
        """
        #rotate every angle so that lower is placed on the origin
        lower0 = lower 
        ang   -= lower ; upper -= lower ; lower = 0
        
        #represent all angles on (-π,π)
        ang = utils.ROC(ang) ; upper = utils.ROC(upper)
        
        #calc midline of upper and lower, and anti-midline m_bar
        midline = (upper + lower )/ 2
        m_bar = midline - np.pi
        
        #determine what angle to return, based on location
        if   ang >= lower and ang <= upper:  x = ang
        elif ang > upper:                    x = upper
        elif ang < m_bar:                    x = upper
        elif ang <= 0 and ang >= m_bar:      x = lower
        
        return x + lower0
    
    @staticmethod
    def between2Angles(a,b1,b2):
        """ is angle a between angles b1 and b2? the minimum angle"""
        #reduce angle to representation on -180 to 180
        angles= [a,b1,b2]
        angCorr = [utils.ROC(ang) for ang in angles]
        dists = [abs(utils.ROC(a-b1)),abs(utils.ROC(a-b2)),abs(utils.ROC(b1-b2))]
        if np.isclose(dists[2],dists[0] + dists[1]):
            return True
        return False
    
    #make this function opperate recursively when presented with a list? 
    @staticmethod
    def noNans(*args):
        for arg in args:
            if np.isnan(arg):
                return False
        return True
    
    @staticmethod
    def GD(x0,df,params):
        """
        peform gradient decent, given df, a function for calculating the 
        derivate of the function to be optimized. 
        """
        next_x = x0   # We start at x0
        gamma = 0.01  # Step size multiplier
        precision = 0.0001  # Desired precision of result
        max_iters = 10000  # Maximum number of iterations
        
        for _ in range(max_iters):
            current_x = next_x
            next_x = current_x - gamma * df(current_x,params)

            step = next_x - current_x
            if abs(step) <= precision:
                break

        return next_x
    
    def Rz(θ):
        """returns matrix which rotates by theta about positive z-axis"""
        return np.array([[ np.cos(θ), -np.sin(θ)],
                         [ np.sin(θ),  np.cos(θ)]])
    
    




#------------------------------------------------ helper classes -----------------------------------------------------
class Foot:
    """
    a physiologic(-ish) foot segment for the human model
    """
    def __init__(self,footLength):
        #compute the critical points defining the foot geometry
        self.jointOrigin        = np.array([0,0])           #apex of the triangle
        self.footLength         = footLength                #forms the base of the triangle
        self.Hindfoot           = .23*self.footLength        #foot is separated into hindfoot and
        self.forfoot            = footLength-self.Hindfoot  #forfoot
        self.footHeight         = .2*self.footLength        #distance from joint origin to line perpendicular to foot length
        self.EEattachment       = .45*self.footLength       #point where the robot and human attach, the End Effector
        self.effectiveMomentArm = np.sqrt(self.footHeight**2 + self.EEattachment**2)
        
        #publically exposed variables:
        self.len = self.effectiveMomentArm
        self.α = np.arctan(self.EEattachment/self.footHeight) #angle offset from height 
    
    def PlotPts(self,θ):
        """returns points to plot the Foot"""
        #define points
        origin  = self.jointOrigin
        midfoot = np.array([0,-self.footHeight])
        heel    = np.array([-self.Hindfoot,-self.footHeight])
        toe     = np.array([self.forfoot,-self.footHeight])
        EE      = np.array([self.EEattachment,-self.footHeight])
        
        #assemble points for continuous line plot
        pts = np.hstack((midfoot[np.newaxis].T,
                         origin [np.newaxis].T,
                         heel   [np.newaxis].T,
                         toe    [np.newaxis].T,
                         origin [np.newaxis].T,
                         EE     [np.newaxis].T))
        
        #static transform to allign ee appropriately for visualization,
        #doesn't impact jacobian
        β = ((np.pi/2) - self.α)  
        return utils.Rz(θ + β) @ pts 
    
    
class Hip:
    """
    hip class handles the calculation of hip joint torque, and simple joint limits.
    """
    def __init__(self):
        self.θmax = np.pi/180 *  70
        self.θmin = np.pi/180 * -90
        
        #define state information for spline fit here
        #self.θ_offset = 0  #why do we need this?

        #define state information for spline fit here
        self._τE =  -5 
        self._τF =   1

    def τE(self,θh,θk,θa):
        return self._τE
    
    def τF(self,θh,θk,θa):
        return self._τF
        
        
class Knee:
    """
    knee class handles the calculation of knee joint torque, and simple joint limits
    """
    def __init__(self):
        self.θmax = np.pi/180 *  0
        self.θmin = np.pi/180 * -120
        
        #define state information for spline fit here
        self._τE =  -3 
        self._τF =   1

    def τE(self,θh,θk,θa):
        return self._τE
    
    def τF(self,θh,θk,θa):
        return self._τF
        

class Ankle:
    """
    Ankle class handles the calculation of Ankle joint torque, and simple joint limits
    """
    def __init__(self,foot):
        self.foot = foot
        
        #limits from foot flat
        self.θmax = np.pi/180 *  20  + self.foot.α   #dorsi-flexion
        self.θmin = np.pi/180 * -50  + self.foot.α   #plantar-flexion
    
        #define state information for spline fit here
        self._τE =  -3 
        self._τF =   .25

    def τE(self,θh,θk,θa):
        return self._τE
    
    def τF(self,θh,θk,θa):
        return self._τF

class hTauSpace:
    """
    class for managing the torque space of the human model
    """
    def __init__(self,hip,knee,ankle):
        self.hip = hip
        self.knee = knee
        self.ankle = ankle
       
    def corners(self,θh,θk,θa):
        """return the corners of the 3 dimensional torque space rectangular prism"""
        
        #order from diagram - hardcoded
        cs = np.array([[self.hip.τF(θh,θk,θa),self.knee.τF(θh,θk,θa),self.ankle.τF(θh,θk,θa)],  #v0
                       [self.hip.τF(θh,θk,θa),self.knee.τE(θh,θk,θa),self.ankle.τF(θh,θk,θa)],  #v1
                       [self.hip.τE(θh,θk,θa),self.knee.τF(θh,θk,θa),self.ankle.τF(θh,θk,θa)],  #v2
                       [self.hip.τF(θh,θk,θa),self.knee.τF(θh,θk,θa),self.ankle.τE(θh,θk,θa)],  #v3
                       [self.hip.τF(θh,θk,θa),self.knee.τE(θh,θk,θa),self.ankle.τE(θh,θk,θa)],  #v4
                       [self.hip.τE(θh,θk,θa),self.knee.τE(θh,θk,θa),self.ankle.τF(θh,θk,θa)],  #v5
                       [self.hip.τE(θh,θk,θa),self.knee.τF(θh,θk,θa),self.ankle.τE(θh,θk,θa)],  #v6
                       [self.hip.τE(θh,θk,θa),self.knee.τE(θh,θk,θa),self.ankle.τE(θh,θk,θa)]]) #v7
        return cs.T
    
    
    def planeCut(self,θh,θk,θa,n_p):
        """
        given a normal vector n_p defined by (nx,ny,nz) and a torque space rectangular prism calculated
        from (θh,θk,θa) - calculate the verticies of the intersection polygon
        """
        #calculate the corners of the torque space rectangular prism
        cs = self.corners(θh,θk,θa)
        n_p = n_p.flatten()
        
        #systematically enumerate all rectangular prism edges. 
        edges = ["e01","e02","e03","e14","e15","e25","e26","e34","e36","e47","e57","e67"]
        
        #calculate the vertecies of intersection between the zero-torque plane and the 12 prism edges
        verts = []
        for vi in range(0,8):
            for vj in range(0,8):
                edge = "e" + str(vi) + str(vj)
                if edge in edges:
                    #find the intersection point λ between plane and edge eij
                    d = 0                            #zero-torque plane passes through origin. 
                    Vi = cs[:,vi]                    #ith corner vertex
                    eij = cs[:,vj] - cs[:,vi]        #vector defining edge ij
                    
                    #make sure edge doesn't lay in the plane n_p
                    if abs(np.dot(n_p,eij)) < .000005:
                        pass
                    else:
                        λ = (d - np.dot(n_p,Vi)) / (np.dot(n_p,eij))
                    
                        #determine if intersection happens inside the prism
                        if λ >= 0 and λ <= 1:
                            verts.append(Vi + λ * eij)
        
        return np.array(verts)  #[npts,3]
    
    def n_p(self,jac_t):
        """
        calculate the direction of the normal vector in tau space, from the force jacobian 
        """
        n_p = jac_t @ np.array([[0],[0],[1]])
        return n_p  #because of the structure of jac, this is always [1,1,1].T
        
    
    def prismPlot(self,θh,θk,θa):
        """return points for plotting torque-space rectangular prism"""
        cs = self.corners(θh,θk,θa)
        pts = np.vstack((cs[:,0],cs[:,1],cs[:,4],cs[:,7],
                         cs[:,7],cs[:,6],cs[:,3],cs[:,0],
                         cs[:,0],cs[:,2],cs[:,5],cs[:,7],
                         cs[:,7],cs[:,4],cs[:,3],cs[:,0],
                         cs[:,0],cs[:,1],cs[:,5],cs[:,7],
                         cs[:,7],cs[:,6],cs[:,2],cs[:,0],))
        
        return pts[:,0], pts[:,1], pts[:,2] 
        
    
    def polygonPlot(self,θh,θk,θa,n_p):
        """
        return plots for vertecies of 
        """
        verts = self.planeCut(θh,θk,θa,n_p)
        hull = ConvexHull(verts)
        verts = verts[hull.vertices,:]
        xs = verts[:,0] ; ys = verts[:,1]
        return xs,ys
        
    

#------------------------------------------------ Human -----------------------------------------------------    

class Human:
    def __init__(self):
        #segment lengths
        self.femur      = .4075    #[meters - from anthopometric tables] 
        self.shank      = .390     #[meters] 
        self.footLength = .255     #[meters]
        self.foot = Foot(self.footLength)
        self.reach = self.femur + self.shank + self.foot.len
        
        #joints and torque space
        self.hip = Hip()
        self.knee = Knee()
        self.ankle = Ankle(self.foot)
        self.τs = hTauSpace(self.hip,self.knee,self.ankle)
        
        #complex limits
        self.αMin = np.pi/180 * -75
        self.αMax = np.pi/180 *  0
        
        #tuning vars for Jch
        self.h0 = -.50; self.m_h = -.8
        self.k0 = -1.79; self.m_k = -1.0; self.ang = 0

        #state:
        self.θh = 0
        self.θk = 0
        self.θa = 0 
        
        #caches
        self.samplePoints = []  
        self.fsCache = dict()    

        
    #------------- private functions --------------------
    def _IK_RR(self,x,y,l1,l2,elbow):
        """
        perform Inverse Kinematics on an RR robot, see lect5 p.9-10 
        """
        #solve for intermediate values
        r = utils.dist(0,0,x,y) - 1e-10
        β = np.arccos((l1**2 + l2**2 - r**2) / (2*l1*l2))
        γ = np.arccos((r**2 + l1**2 - l2**2) / (2*r*l1))
        α = np.arctan2(y,x)
        
        #handle different elbow states
        θ1 = α - γ
        θ2 = np.pi - β
        
        if elbow == "down":
            return θ1,θ2
        elif elbow == "up":
            θ1pm = θ1 + 2*γ  
            θ2pm = -θ2   
            return θ1pm,θ2pm

    
    def _IK_RRR_θa(self,x,y,θ3):
        """
        solves the IK problem given an ankle joint locked at angle θa 
        """
        #define local vars: 
        L1 = self.femur
        L2 = self.shank
        L3 = self.foot.len
        
        #determine the ankle state based on θ3  (consider refactoring into a property?)
        θ3 = utils.ROC(θ3,lower = 0,upper = 2*np.pi)
        if θ3 >= 0 and θ3 < np.pi:
            wrist = "down"
        if θ3 >= np.pi and θ3 <= 2*np.pi:
            wrist = "up"
    
        #determine Lf, the length of the ficticious segment
        a = np.pi - θ3
        Lf = (L2**2 + L3**2 - 2*(L2)*(L3)*np.cos(a))**.5
        
        #determine the IK for the RR robot formed by L1 and the fictitious Lf 
        θ1, θ2f = self._IK_RR(x,y,L1,Lf,"up")
        
        #determine the FK to the first link 
        x1 = L1*np.cos(θ1); y1 = L1*np.sin(θ1)
        
        #determine the IK from the tip of the first link, to the end effector location
        θ2, θ3 = self._IK_RR(x - x1,y - y1,L2,L3,wrist)
        θ2 -= θ1                    #angle correction for different coordinate frames 
        
        return θ1,θ2,θ3
        
    
    def IK(self,x,y,method = "jch"):
        """
        given a location x,y of the midfoot, what are the joint angles required to
        achieve that position? 
        """
        #check that IK can be performed
        ϵ = .001
        if utils.dist(0,0,x,y) + ϵ > self.femur + self.shank + self.foot.len:
            return np.nan,np.nan,np.nan
        
        #locked ankle
        elif method == "locked-ankle":
            θh,θk,θa = self._IK_RRR_θa(x,y,0)
            return θh,θk,θa
        
        #joint-coupling huristic
        elif method == "jch":
            neutral = self.foot.α
            θh,θk,θa = self._IK_RRR_θa(x,y,0)
            if not utils.noNans(θh,θk,θa):
                return np.nan,np.nan,np.nan
            hip2AnkleAngle = np.arctan2(y,x)
            self.ang = neutral + self.h0 + self.m_h * hip2AnkleAngle + self.k0 + self.m_k * θk
            self.ang = utils.clampAng(self.ang,self.ankle.θmin,self.ankle.θmax)
            θh,θk,θa = self._IK_RRR_θa(x,y,self.ang)
            return θh,θk,θa
        
            
        else:
            raise ValueError("method {} not yet implemented".format(method))

    
    def _FK(self,θh,θk,θa):
        """
        calculate the forward kinematics of each joint and the contact point
        returns: 
            vectors of x and y coordinates of forward kinematic
            [hip,knee,ankle,ball]
        """
        #calculate the sines and cosines of successive angles
        s_h   = np.sin(θh)        ; c_h   = np.cos(θh)
        s_hk  = np.sin(θh+θk)     ; c_hk  = np.cos(θh+θk)
        s_hka = np.sin(θh+θk+θa)  ; c_hka = np.cos(θh+θk+θa)
        
        #perform the forward kinematics 
        hip     = [0,0]
        knee    = [  hip[0] + self.femur*c_h ,      hip[1] + self.femur*s_h]                                     
        ankle   = [ knee[0] + self.shank*c_hk ,    knee[1] + self.shank*s_hk]
        ball    = [ankle[0] + self.foot.len*c_hka,ankle[1] + self.foot.len*s_hka]
        
        #reorganize into vectors
        x = [hip[0],knee[0],ankle[0],ball[0]]
        y = [hip[1],knee[1],ankle[1],ball[1]]
        
        return (x,y)
 
    
    def _Jac(self,θh,θk,θa,mode = "force"):
        """
        calculate the force or (velocity) jacobian from at a particular leg state
        by convention, jac is velocity jacobian jac_t is force jacobian, and 
        jinv_t is the inverse force space jacobian, etc. 
        """
        #the jacobian is expressable in terms of the forward kinematic quantities
        # duffy p.115
        xs,ys = self._FK(θh,θk,θa)
        xh,xk,xa,xb = xs[0],xs[1],xs[2],xs[3]
        yh,yk,ya,yb = ys[0],ys[1],ys[2],ys[3]
        
        
        #calculate the force jacobian 
        #               x(θ's)     y(θ's)    ϕ(θ's)
        jac = np.array([[-yb,        xb,       1],    #θh
                        [-(yb-yk),   xb-xk,    1],    #θk
                        [-(yb-ya),   xb-xa,    1]])   #θa
        
        if mode == "force":
            return jac
        elif mode == "velocity":
            return jac.T
        
        
    def reachableSpacePlot(self):
        """
        plot the outer reach of the leg,without regard to joint limits
        """
        t = np.linspace(0,2* np.pi,100)
        r = self.reach
        x = r * np.cos(t)
        y = r * np.sin(t)
        return x,y
    
         
    def Sample(self,samplingMode = "Cartesian", limits = []):
        """
        return a sampling of points, ([xs],[ys]) within the reachable space
        of the human model, subject to limit conditions that constrain the 
        sampled space
        inputs:
            sampling mode - FK, cartesian, polar
            limits - list of limiting fxn names, as strings.
        """
        
        if samplingMode == "FK":
            #calculate the set of points using sampling methods
            nh = 50; nk = 25; npts = nh * nk
            pts = np.zeros((nh,nk,2))
            θhs = np.linspace(self.hip.θmin,self.hip.θmax,nh)
            θks = np.linspace(self.knee.θmin,self.knee.θmax,nk)
            
            #sample the space - within joint constraints
            for i,θh in enumerate(θhs):
                for j,θk in enumerate(θks):
                    x,y = self._FK(θh,θk,self.ankle.θmin)
                    if self.limits(x,y,θh,θk,self.ankle.θmin,limits):
                        pts[i,j,:] = np.array((x[3],y[3]))

            #flatten pts (depending on if knee or hip is first)
            inner = "knee"
            if inner == "hip" : pts = pts.reshape(nh*nk,2,order = "F")
            if inner == "knee": pts = pts.reshape(nh*nk,2,order = "C")
        
        if samplingMode == "Cartesian":
            dx = .05   #x -sampling density
            dy = .05   #y -sampling density
            xs = np.arange(-self.reach,self.reach + dx,dx)
            ys = np.arange(-self.reach,self.reach + dy,dy)
            npts = xs.shape[0] * ys.shape[0]
            pts = np.zeros((xs.shape[0],ys.shape[0],2))
            
            #sample the space - within joint constraints
            for i,x in enumerate(xs):
                for j,y in enumerate(ys):
                    θh,θk,θa = self.IK(x,y)
                    if self.limits(x,y,θh,θk,θa,limits):
                        pts[i,j,:] = np.array((x,y))
                        
                        
        if samplingMode == "polar":
            pass
        
            
        #flatten pts (depending on if knee or hip is first)
        inner = "y" 
        if inner == "x" : pts = pts.reshape(npts,2,order = "F")
        if inner == "y":  pts = pts.reshape(npts,2,order = "C")
                                         
        #pop nans
        pts = pts[~np.isnan(pts).any(axis=1)]
        
        #cache list
        self.samplePoints = pts  #keep as a matrix? 

        #return
        return pts[:,0] , pts[:,1]
    
    
    
    def limits(self,x,y,θh,θk,θa,lims = []):
        """
        checks if a point satisfies the selected limit functions. 
        if it does, return True, else return false. 
        input:
            IK and FK vars
            lims - list of numbers, with indecies of which tests to run
        """
        #0 - can IK even be performed?
        if 0 in lims:
            θs = self.IK(x,y)
            if not utils.noNans(θs[0],θs[1],θs[2]):
                return False

        #1 - simple joint limits
        if 1 in lims:
            if not utils.between2Angles(θh,self.hip.θmin,self.hip.θmax) or \
               not utils.between2Angles(θk,self.knee.θmin,self.knee.θmax):
                return False
        
        #2 - alpha method - angle between hip and ankle joint
        if 2 in lims:
            xs,ys = self._FK(θh,θk,θa)
            α = np.arctan2(ys[2],xs[2])
            if not utils.between2Angles(α,self.αMin,self.αMax):
                return False
        
        #3 - combined joint limits
        if 3 in lims:
            pass
        
        return True
            
    def FK(self,θh,θk,θa):
        """
        given the joint angles θh,θk,θa, where is the middle of the foot located in space?
        """
        x,y = self._FK(θh,θk,θa) ; return (x[3],y[3])
    
    def FKplot(self,θh,θk,θa):
        """
        forward kinematic plotting vector for viewing human leg
        """
        _xs,_ys = self._FK(θh,θk,θa)
        ϕ = θh + θk + θa
        fxs,fys = self.foot.PlotPts(ϕ)
        fxs += _xs[2] ; fys += _ys[2]
        xs = np.hstack((np.array(_xs[:-1]),fxs))
        ys = np.hstack((np.array(_ys[:-1]),fys))
        return xs,ys
        
        
    def FS(self,x,y):
        """
        evaluate the Force Space of the human at a particular location location in the saggital plane
        and return a shape object that can be used to score FS overlap
        """
        θh,θk,θa, = self.IK(x,y)
        J_T = self._Jac(θh,θk,θa)
        J_invT = np.linalg.inv(J_T)
        cs = J_invT @  self.τs.corners(θh,θk,θa)
        fs = cs[0:2,:]
        return fs
    
    def FSpolygon(self,x,y):
        """
        generate the force space polygon
        """
    
    def FSplot(self,x,y,Scale):
        """
        given an endeffector location (x,y), return the xs and ys for plotting
        the force polygon
        """
        fs = self.FS(x,y) * Scale
        hull = ConvexHull(fs.T)
        verts = fs.T[hull.vertices,:]
        fs = verts.T
        xs = fs[0,:] ; ys = fs[1,:]
        return xs,ys
    
    
    def buildFScache(self):
        """
        run this function to build a cache (dictionary) of FS polygons which can be
        looked up via their x and y location with O(1) performance. 
        """
        pass
            


#-------------------- main ----------------
h = Human()
θh,θk,θa = h.IK(.61693,-.02307,method = "jch")
#θh,θk,θa = h.IK(.75,0,method = "jch")
#jac_t = h._Jac(θh,θk,θa)
#n_p = h.τs.n_p(jac_t)
#verts = h.τs.planeCut(θh,θk,θa,n_p)
#

#%%

from scipy.interpolate import interp1d
import numpy as np

# objective function
class testObjFxn:
    def __init__(self):
        self.X = np.linspace(0,1,1000)
        self._y = np.asarray([self.objective(x) for x in self.X])
        self.f =  interp1d(self.X, self._y,kind='cubic')
        
    def objective(self,x, noise=0.1):
        noise = np.random.normal(loc=0, scale=noise)
        return (x**2 * np.sin(5 * np.pi * x)**6.0) + noise
    


runTests = True


class SA:
    def __init__(self,boxConstraints):
        self.nIter = 1000;      #total number of iterations to perform
        self.dist = "uniform"   #neighborhood sampling method 
        self.AnnealMode = "exp" #Annealing mode ["exp","linear","stepped"] 
        self.T = 1              #temperature variable on [1-0)
        self.N = 1              #neighborhood size coefficient [1-0)
        self.C_half = 200       #half-life cooling              [trials]
        self.N_half = 150       #half-life of neighborhood size [trials]
        self.fxBest = 0         #best encountered function value
        self.xBest = []         #location of best value
        bcs = boxConstraints    # problem box constraints
        self.x_max = bcs[:,0]   #upper limits on box constraints for x variable
        self.x_min = bcs[:,1]   #lower limits on box constraints for x variable
        
    def _center(self,x_max,x_min):
        """
        find  the center of a box defined by limits
        """
        return [(x_max[i] + x_min[i])/2 for i in range(len(x_max))]
    
    def sample(self,x):
        """
        sample the space for the next sample based on the neighborhood
        function
        input:
            x
        output:
            x_new
        """
        #define the (ever-shrinking) neighborhood bounding box
        nx_max = (self.N * self.x_max) + x
        nx_min = (self.N * self.x_min) + x
        
        #define the combined bounding box as an intersection between
        #the box constraints and the neighborhood box - a mini-max problem
        for i in range(len(self.x_max)):
            cx_max = min(self.x_max,nx_max)
            cx_min = max(self.x_min,nx_min)
        
        #sample from the combined box, according to the appropriate distribution
        if self.dist == "uniform": 
            x_new = np.random.uniform(cx_min,cx_max) #vectorized
            
        if self.dist == "normal":
            pass
#             zScore = 1.645 #95% confidence interval
#             x_new = []
#             while len(x_new) > len(self.x_min):
#                 x = np.random.normal(loc=0.0, scale=1.0, size=None)
#                 if abs(x) < zScore:
#                     x_new.append(x)
#             c_range = cx_max - cx_min
            
 
        return x_new

    def optimize(self,obj):
        x0 =  self._center(self.x_max,self.x_min)
        fx0 = obj(x0)
        
        for n in range(self.nIter):
            x = self.sample(x0)
            print(x)
            fx = obj(x)
            
            #if better, always keep
            if fx > fx0:
                fx0 = fx ; x0 = x
                
            #maintain best (for restarts, if used)
            if fx > self.fxBest:
                self.fxBest = fx ; self.xBest = x 
                
            #probabalistically accept worse answer
            else:
                Δf = fx - fx0
                r = np.random.random()
                if r < np.exp(-Δf/self.T):
                    fx0 = fx ; x0 = x
                else:
                    pass
            
            #decrease temperature and neighborhood size
            self.T = .5**(n/self.C_half)
            self.N = .5**(n/self.N_half)
        
        return x0,fx0
                    
 





if runTests:
    #plot
    tester = testObjFxn()
    boxConstraints = np.array([[.9,.1]])
    sa = SA(boxConstraints)

    sa.optimize(tester.f)
    
    #%timeit sa.optimize(test.f)









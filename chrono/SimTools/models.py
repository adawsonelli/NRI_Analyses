"""
library of functions for constructing chrono models
"""

#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np
from SimTools.utils import Rx

# Change this path to asset path, if running from other working dir. 
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chronoDataDir = "D:/programFiles/Miniconda3/pkgs/pychrono-5.0.0-py37_9/Library/data/"
chrono.SetChronoDataPath(chronoDataDir)
assetsPath = "C:/Users/adaws/Documents/gitRepos/NRI_Analyses/chrono/assets/"


#----------------------------- classes ----------------------------------------
class ALEXR: 
    """
    contains the state information of the ALEX Robot and functions to calculate
    it's inverse kinematics
    """
    def __init__(self):
        #hardcode robot link lengths and origins
        self.xl = 0 ; self.yl = -.27 
        self.xr = 0 ; self.yr = -.055
        
        
        #left robot
        self.L1l = .48                  #meters
        self.L2l = .28                  #meters
        self.L3l = .38                  #meters
        self.L23l = self.L2l + self.L3l #the angle between them is 0, therfore they may be treated as the same link
        
        
        #right robot
        self.L1r = .28               #the right robot is a normal RR robot
        self.L2r = .40
    
    #inverse kinematics equations for an RR robot
    @staticmethod
    def IK_RR(x,y,l1,l2,elbow):
        """
        perform Inverse Kinematics on an RR robot, see lect5 p.9-10 
        """
        #solve for intermediate values
        r = np.linalg.norm((x,y)) - 1e-12 #solves numerical issue at θ2 == 0 
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
        
    def IK_2DOF(self,Xee,Yee,side = "right"):
        """
        calculate the IK for the ALEXR robot, given the side (which sets the elbow config)
        inputs:
            Xee - the x location of the end effector
            Yee - the y location of the end effector
            side - is the robot on the right or left side of the vertical meridian? this 
                   changes the relationships between the discrete vars how the elbow states look
        outputs:
            θ1l , θ2l, θ1r , θ2r
        """
        
        #change the elbow vars based on the discrete var - side (of which there are 2)
        if side == "right":
            ef = "up"
            e3 = "down"
        elif side == "left":
            ef = "down"
            e3 = "up"
        
        #calculate IK robot angles (this version is simplified from the Gen5bl to the case where θ=0)
        θ1l , θ2l = ALEXR.IK_RR(Xee - self.xl,Yee - self.yl,self.L1l,self.L23l,ef)     # IK of the left robot to end effector location
        Jx = Xee - self.L3l*np.cos(θ1l + θ2l)                                          # locations of the joint in 3D space 
        Jy = Yee - self.L3l*np.sin(θ1l + θ2l)                                          # (vector subtraction method)
        θ1r , θ2r = ALEXR.IK_RR(Jx - self.xr,Jy - self.yr,self.L1r,self.L2r,e3)        # IK of the right robot
    
        #return full IK angles
        return θ1l , θ2l, θ1r , θ2r
    
    def feasable_point(self,Xee,Yee,side = "right"):
        """
        return's true if IK solves, false if it does not. 
        """
        θ1l , θ2l, θ1r , θ2r = self.IK_2DOF(Xee, Yee,side)
        if np.isnan(θ1l and θ2l and θ1r and θ2r):
            return False
        else:
            return True
        
    


#------------------- public facing functions ----------------------------------    
def add_θ1l_joint(system,joint):
    """
    adds the type of joint (either driven or non-driven) at the θ1l location
    """
    a = ALEXR()                                                               # contains robot state information
    system.refs["GB<->L1l"] = joint                                           # maintain a reference to the joint
    local = True                                                              # we will use the local frame
    GB_j_r =  chrono.ChFrameD(chrono.ChVectorD(0,-1*(a.yr - a.yl)/2,0.01))    # GB  attachment point j represented in ref frame (same as COG frame)
    L1l_j_r = chrono.ChFrameD(chrono.ChVectorD(-1*a.L1l/2,0,0))               # L1l attachment point j represented in ref frame
    GB  = system.refs["GB"]                                                   # get a reference to the ground body
    L1l = system.refs["L1l"]                                                  # get a reference to Link 1 body
    GB_j_COG = GB_j_r                                                         # COG isn't displaced in GB
    L1l_j_COG = L1l_j_r  >> L1l.GetFrame_REF_to_COG()                         # express GB<->L1l joint relative to L1l COG frame
    joint.Initialize(GB,L1l,local,GB_j_COG,L1l_j_COG)                           # init joint
    system.Add(joint) 
    
    
    
    
def add_θ1r_joint(system,joint):
    """
    adds the type of joint (either driven or non-driven) at the θ1r location
    """
    a = ALEXR()                                                               # contains robot state information
    system.refs["GB<->L1r"] = joint                                           # maintain a reference to the joint
    local = True                                                              # we will use the local frame
    GB_j_r  =  chrono.ChFrameD(chrono.ChVectorD(0,(a.yr - a.yl)/2,.02))       # GB  attachment point j represented in ref frame (same as COG frame)
    L1r_j_r = chrono.ChFrameD(chrono.ChVectorD(-1*a.L1r/2,0,0))               # L1l attachment point j represented in ref frame
    GB  = system.refs["GB"]                                                   # get a reference to the ground body
    L1r = system.refs["L1r"]                                                  # get a reference to Link 1 body
    GB_j_COG = GB_j_r                                                         # COG isn't displaced in GB
    L1r_j_COG = L1r_j_r  >> L1r.GetFrame_REF_to_COG()                         # express GB<->L1l joint relative to L1l COG frame
    joint.Initialize(GB,L1r,local,GB_j_COG,L1r_j_COG)                         # init joint
    system.Add(joint)   
    


def buildALEXR(system,
               Xee  = .5,
               Yee  = -.5,
               eeMass = 1,
               side = "right"):
    """
    public interface for building the ALEXR
    
    :param system: the Chrono system to which the ALEXR robot is added
    """
    
    #--------------- state infromation for initialization ---------------------
    
    #initial location of the end effector (EE) of the ALEX Robot
    #Xee =  .5
    #Yee = -.5
    
    #specify mass properties of the payload at the end effector. 
    #eeMass = 1   
    
    
    #set the elbow discrete vars this will flip for right vs. left side useage
    #side = "right"  # "left"
    
    
    #----------- Calculate IK angles using custom Library ---------------------
    a = ALEXR()        #encodes robot state info
    system.refs = {}   #setup the dictionary of object references
    
    #ALEXR link lengths and origins
    xl = a.xl ; yl = a.yl 
    xr = a.xr ; yr = a.yr
    
    #left robot
    _L1l = a.L1l            
    _L2l = a.L2l            
    _L3l = a.L3l            
    _L23l= a.L23l   
    
    #right robot
    _L1r = a.L1r              
    _L2r = a.L2r
        
    #calculate IK robot angles 
    θ1l , θ2l, θ1r , θ2r = a.IK_2DOF(Xee,Yee,side)
   
    
    #perform a check here, and throw an error if the original configuration can't be solved.
    if not a.feasable_point(Xee, Yee,side):
        raise ValueError("there is no solution to IK for the end-effector location specified (Xee,Yee)")
    
    
    #--------------- create each link as a rigid body -------------------------
    
    # chrono uses a right handed coordinate system, and the model is constructed to look like the plotly model,
    # but the irrelict visualizer operates in the mirror world - with a left handed coordinate system
    # the long axis of the link to the right is the x axis, up is y, z is link rotation direction out of the page
    # R is the rotation matrix between the default link frame orientation, and the solidworks coordinate system 
    # R (solidworks - > link frame)

    #------------- ground body ------------
    GB = chrono.ChBodyAuxRef()
    GB.SetPos(chrono.ChVectorD(0,(yl+yr)/2,0))
    GB.SetBodyFixed(True)
    
    #set mesh visualization
    mesh_for_visualization = chrono.ChTriangleMeshConnected()
    mesh_for_visualization.LoadWavefrontMesh(assetsPath +'ground.obj')
    
    # Optionally: you can scale/shrink/rotate/translate the mesh using this:
    meshRotation = chrono.ChMatrix33D(np.pi/2,chrono.ChVectorD(0,1,0))
    mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), meshRotation)
    
    # Now the  triangle mesh is inserted in a ChTriangleMeshShape visualization asset, 
    # and added to the body
    visualization_shape = chrono.ChTriangleMeshShape()
    visualization_shape.SetMesh(mesh_for_visualization)
    GB.AddAsset(visualization_shape)
    system.Add(GB)
    system.refs["GB"] = GB
    
    
    #--------- coordinate frame ---------------
    coord = chrono.ChBodyAuxRef()
    coord.SetPos(chrono.ChVectorD(0,0,0))
    coord.SetBodyFixed(True)
    
    mesh_for_visualization = chrono.ChTriangleMeshConnected()
    mesh_for_visualization.LoadWavefrontMesh(assetsPath +'coords.obj')
    mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), chrono.ChMatrix33D(.01))
    
    visualization_shape = chrono.ChTriangleMeshShape()
    visualization_shape.SetMesh(mesh_for_visualization)
    coord.AddAsset(visualization_shape)
    system.Add(coord)
    
    
    #----------- left link 1 ------------------
    #add body
    L1l = chrono.ChBodyAuxRef()
    L1l.SetBodyFixed(False)
    system.Add(L1l)
    system.refs["L1l"] = L1l
    
    
    #set position,orientation with FK (while REF frame and COG frame are coincident)
    x =  xl + (_L1l/2)*np.cos(θ1l)
    y =  yl + (_L1l/2)*np.sin(θ1l)
    L1l.SetPos(chrono.ChVectorD(x,y,.01))
    L1l.SetRot(chrono.ChMatrix33D(θ1l,chrono.ChVectorD(0,0,1)))
    
    #add visualization
    mesh_for_visualization = chrono.ChTriangleMeshConnected()
    mesh_for_visualization.LoadWavefrontMesh(assetsPath +'_L1l.obj')
    meshRotation = chrono.ChMatrix33D(np.pi/2,chrono.ChVectorD(0,1,0))
    mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), meshRotation)
    
    visualization_shape = chrono.ChTriangleMeshShape()
    visualization_shape.SetMesh(mesh_for_visualization)
    L1l.AddAsset(visualization_shape)
    
    texture = chrono.ChTexture()
    texture.SetTextureFilename(assetsPath + 'blue.png')
    L1l.GetAssets().push_back(texture)
    
    #set the mass and inertial properties
    #              xs   ys   zs
    R1 = np.array([[ 0,   0,  -1],    #xl                         #found by hand
                   [ 0,  -1,   0],    #yl
                   [-1,   0,   0]])   #zl
    
    L1l.SetMass(3.642)
    Is = np.array([[ 0.13286460, -0.00001280, 0.03326759],        # centroidal moment of inertia
                   [-0.00001280,  0.15328523,-0.00014996],
                   [ 0.03326759, -0.00014996, 0.03071782]]) 
    Il = R1 @ Is @ np.linalg.inv(R1)                              # rotate the inertia tensor into the link frame
    Ilch = chrono.ChMatrix33D()
    Ilch.SetMatr(Il.tolist())
    L1l.SetInertia(Ilch)
    
    # move the COG frame
    c = R1 @ np.array([[.0904],[-.0004],[.1461]])
    L1l.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(c[0,0],c[1,0],c[2,0])))
    
    
    
    #----------- left link 2 ------------------
    #add body
    L2l = chrono.ChBodyAuxRef()
    L2l.SetBodyFixed(False)
    system.Add(L2l)
    system.refs["L2l"] = L2l
    
    #set position,orientation with FK
    x =  xl + (_L1l)*np.cos(θ1l) + (_L23l/2)*np.cos(θ1l + θ2l)
    y =  yl + (_L1l)*np.sin(θ1l) + (_L23l/2)*np.sin(θ1l + θ2l)
    L2l.SetPos(chrono.ChVectorD(x,y,.02))
    L2l.SetRot(chrono.ChMatrix33D(θ1l + θ2l,chrono.ChVectorD(0,0,1)))
    
    #add visualization
    mesh_for_visualization = chrono.ChTriangleMeshConnected()
    mesh_for_visualization.LoadWavefrontMesh(assetsPath +'_L2l.obj')
    meshRotation = chrono.ChMatrix33D(np.pi/2,chrono.ChVectorD(0,1,0))
    
    #mesh origin was slightly off, so I hand tuned it 
    mesh_for_visualization.Transform(chrono.ChVectorD(-.00775,0,0), meshRotation)
    
    visualization_shape = chrono.ChTriangleMeshShape()
    visualization_shape.SetMesh(mesh_for_visualization)
    L2l.AddAsset(visualization_shape)
    
    texture = chrono.ChTexture()
    texture.SetTextureFilename(assetsPath + 'blue.png')
    L2l.GetAssets().push_back(texture)
    
    #set the mass and inertial properties
    #                xs   ys   zs
    R2 = np.array([[ 0,   0,   1],    #xl                         #found by hand
                   [ 0,   1,   0],    #yl
                   [-1,   0,   0]])   #zl
    L2l.SetMass(1.158)
    Is = np.array([[ 0.04061717,  0.00000000, 0.00000000],     # centroidal moment of inertia
                   [ 0.00000000,  0.04040908, 0.00000000],
                   [ 0.00000000,  0.00000000, 0.00072961]]) 
    Il = R2 @ Is @ np.linalg.inv(R2)                             # rotate the inertia tensor into the link frame
    Ilch = chrono.ChMatrix33D()
    Ilch.SetMatr(Il.tolist())
    L2l.SetInertia(Ilch)
    
    # move the COG frame
    c = R2 @ np.array([[0],[0],[-.1192]])
    L2l.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(c[0,0],c[1,0],c[2,0])))
    
    
    #----------- right link 1 -----------------
    #add body
    L1r = chrono.ChBodyAuxRef()
    L1r.SetBodyFixed(False)
    system.Add(L1r)
    system.refs["L1r"] = L1r
    
    #set position,orientation with FK
    x =  xr + (_L1r/2)*np.cos(θ1r)
    y =  yr + (_L1r/2)*np.sin(θ1r)
    L1r.SetPos(chrono.ChVectorD(x,y,.02))
    L1r.SetRot(chrono.ChMatrix33D(θ1r,chrono.ChVectorD(0,0,1)))
    
    #add visualization
    mesh_for_visualization = chrono.ChTriangleMeshConnected()
    mesh_for_visualization.LoadWavefrontMesh(assetsPath +'_L1r.obj')
    meshRotation = chrono.ChMatrix33D(np.pi/2,chrono.ChVectorD(0,1,0))
    mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), meshRotation)
    
    visualization_shape = chrono.ChTriangleMeshShape()
    visualization_shape.SetMesh(mesh_for_visualization)
    L1r.AddAsset(visualization_shape)
    
    texture = chrono.ChTexture()
    texture.SetTextureFilename(assetsPath + 'red.png')
    L1r.GetAssets().push_back(texture)
    
    #set the mass and inertial properties
    L1r.SetMass(4.1637)
    Is = np.array([[ 0.05261769, -0.00006255, 0.02546226],     # centroidal moment of inertia
                   [-0.00006255,  0.09428792,-0.00007718],
                   [ 0.02546226, -0.00007718, 0.05243999]]) 
    Il = R1 @ Is @ np.linalg.inv(R1)                           # rotate the inertia tensor into the link frame
    Ilch = chrono.ChMatrix33D()
    Ilch.SetMatr(Il.tolist())
    L1r.SetInertia(Ilch)
    
    # move the COG frame
    c = R1 @ np.array([[0.1222],[-0.0004],[.0927]])
    L1r.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(c[0,0],c[1,0],c[2,0])))
    
    #----------- right link 2 -----------------
    #add body
    L2r = chrono.ChBodyAuxRef()
    L2r.SetBodyFixed(False)
    system.Add(L2r)
    system.refs["L2r"] = L2r
    
    #set position,orientation with FK
    x =  xr + (_L1r)*np.cos(θ1r) + (_L2r/2)*np.cos(θ1r + θ2r)
    y =  yr + (_L1r)*np.sin(θ1r) + (_L2r/2)*np.sin(θ1r + θ2r)
    L2r.SetPos(chrono.ChVectorD(x,y,.03))
    L2r.SetRot(chrono.ChMatrix33D(θ1r + θ2r,chrono.ChVectorD(0,0,1)))
    
    #add visualization
    mesh_for_visualization = chrono.ChTriangleMeshConnected()
    mesh_for_visualization.LoadWavefrontMesh(assetsPath +'_L2r.obj')
    meshRotation = chrono.ChMatrix33D(np.pi/2,chrono.ChVectorD(0,1,0))
    mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), meshRotation)
    
    visualization_shape = chrono.ChTriangleMeshShape()
    visualization_shape.SetMesh(mesh_for_visualization)
    L2r.AddAsset(visualization_shape)
    
    texture = chrono.ChTexture()
    texture.SetTextureFilename(assetsPath + 'red.png')
    L2r.GetAssets().push_back(texture)
    
    #set the mass and inertial properties
    L2r.SetMass(1.1947)
    Is = np.array([[ 0.06453132,  0.00000000, 0.00101029],     # centroidal moment of inertia
                   [ 0.00000000,  0.06454599, 0.00000000],
                   [ 0.00101029,  0.00000000, 0.00093856]]) 
    Il = R1 @ Is @ np.linalg.inv(R1)      #R1 is correct here, I checked, rotate the inertia tensor into the link frame
    Ilch = chrono.ChMatrix33D()
    Ilch.SetMatr(Il.tolist())
    L2r.SetInertia(Ilch)
    
    # move the COG frame
    c = R1 @ np.array([[-0.0041],[0.0000],[-0.0499]])
    L2r.SetFrame_COG_to_REF(chrono.ChFrameD(chrono.ChVectorD(c[0,0],c[1,0],c[2,0])))
    
    #----------- end effector payload ---------
    #add body
    ee = chrono.ChBodyAuxRef()
    ee.SetBodyFixed(False)
    system.Add(ee)
    system.refs["EE"] = ee
    
    #add mass properties  //improve these based on actual data...
    ee.SetMass(eeMass) 
    #can leave the inertia large, as this frame doesn't rotate (it can be thought of as on a bearing)
    
    #set position,orientation with FK
    x =  xl + (_L1l)*np.cos(θ1l) + (_L23l)*np.cos(θ1l + θ2l)
    y =  yl + (_L1l)*np.sin(θ1l) + (_L23l)*np.sin(θ1l + θ2l)
    ee.SetPos(chrono.ChVectorD(x,y,.03))
    ee.SetRot(chrono.ChMatrix33D(0,chrono.ChVectorD(0,0,1)))
    
    #add visualization
    mesh_for_visualization = chrono.ChTriangleMeshConnected()
    mesh_for_visualization.LoadWavefrontMesh(assetsPath +'_EE.obj')
    meshRotation = chrono.ChMatrix33D(np.pi/2,chrono.ChVectorD(0,1,0))
    mesh_for_visualization.Transform(chrono.ChVectorD(0,0,0), meshRotation)
    
    visualization_shape = chrono.ChTriangleMeshShape()
    visualization_shape.SetMesh(mesh_for_visualization)
    ee.AddAsset(visualization_shape)
    
    
    #----------------------- create the revolute joints ---------------------------
    # joint frame naming conventions
    # X_c_a
    #   X   - body the frame is attached to in the joint
    #   c_a - c frame represented in the a frame
    # potential frames
    #   j   - the joint frame - where the joint marker and frame is located
    #   ref - the reference frame located at the center of the link
    #   cog - the cog of the link, which is offset from the reference frame in an ChAuxRefBody()
    # example L1l_j_cog
    #   this refers to a frame on body L1l, attached at the joint location, represented 
    #   relative to the COG of L1l. this is the objective, as joints must be formed relative to 
    #   a bodies COG frame, not it's auxillary reference frame
    
    #------------- GB  <-> L1l --------------
#    jt = chrono.ChLinkRevolute()                                        #set higher up 
#    add_θ1l_joint(system,jt)         
#     
    ##------------- L1l <-> L2l --------------
    jt = chrono.ChLinkRevolute()                                         # create revolute joint object                       
    local = True                                                         # we will use the local frame
    L1l_j_r = chrono.ChFrameD(chrono.ChVectorD(_L1l/2,0,0.01))           # local frame of attachment
    L2l_j_r = chrono.ChFrameD(chrono.ChVectorD(-1*_L23l/2,0,0))          # local frame of attachment
    L1l_j_COG = L1l_j_r  >> L1l.GetFrame_REF_to_COG()                    # express L1l <-> L2l joint relative to L1l COG frame
    L2l_j_COG = L2l_j_r  >> L2l.GetFrame_REF_to_COG()                    # express L1l <-> L2l joint relative to L12 COG frame
    jt.Initialize(L1l,L2l,local,L1l_j_COG,L2l_j_COG)                     # init joint
    system.Add(jt)                                                       # add to system
    system.refs["L1l<->L2l"] = jt                                        # maintain a reference to the joint
    
    ##------------- GB  <-> L1r --------------
#    jt = chrono.ChLinkRevolute()
#    add_θ1r_joint(system,jt)     
    
    
    ##------------- L1r <-> L2r --------------
    jt = chrono.ChLinkRevolute()                                         # create revolute joint object
    local = True                                                         # we will use the local frame
    L1r_j_r = chrono.ChFrameD(chrono.ChVectorD(_L1r/2,0,.01))            # local frame of attachment
    L2r_j_r = chrono.ChFrameD(chrono.ChVectorD(-1*_L2r/2,0,0))           # local frame of attachment
    L1r_j_COG = L1r_j_r  >> L1r.GetFrame_REF_to_COG()                    # express L1l <-> L2l joint relative to L1l COG frame
    L2r_j_COG = L2r_j_r  >> L2r.GetFrame_REF_to_COG()                    # express L1l <-> L2l joint relative to L12 COG frame
    jt.Initialize(L1r,L2r,local,L1r_j_COG,L2r_j_COG)                     # init joint
    system.Add(jt)                                                       # add to system
    system.refs["L1r<->L2r"] = jt                                        # maintain a reference to the joint
    
    ##------------- L2l <-> L2r --------------
    jt = chrono.ChLinkRevolute()                                         # create revolute joint object
    local = True                                                         # we will use the local frame
    dj = -1*(_L23l/2 - _L2l)                                             # distance from center to joint point
    L2l_j_r = chrono.ChFrameD(chrono.ChVectorD(dj,0,.01))                # local frame of attachment
    L2r_j_r = chrono.ChFrameD(chrono.ChVectorD(_L2r/2,0,0))              # local frame of attachment
    L2l_j_COG = L2l_j_r  >> L2l.GetFrame_REF_to_COG()                    # express L2l <-> L2r joint relative to L2l COG frame
    L2r_j_COG = L2r_j_r  >> L2r.GetFrame_REF_to_COG()                    # express L2l <-> L2r joint relative to L2r COG frame
    jt.Initialize(L2l,L2r,local,L2l_j_COG,L2r_j_COG)                     # init joint
    system.Add(jt)                                                       # add to system
    system.refs["L2l<->L2r"] = jt                                        # maintain a reference to the joint
    #
    ##------------- ee <-> L2l --------------
    jt = chrono.ChLinkRevolute()                                         # create revolute joint object
    local = True                                                         # we will use the local frame
    L2l_j_r = chrono.ChFrameD(chrono.ChVectorD(_L23l/2,0,.01))           # local frame of attachment
    ee_j_r = chrono.ChFrameD(chrono.ChVectorD(0,0,0))                    # local frame of attachment                                                       # COG isn't displaced in GB
    L2l_j_COG = L2l_j_r  >> L2l.GetFrame_REF_to_COG()                    # express ee <-> L2l joint relative to L1l COG frame
    ee_j_COG = ee_j_r                                                    # COG isn't displaced in ee frame
    jt.Initialize(L2l,ee,local,L2l_j_COG,ee_j_COG)                       # init joint
    system.Add(jt)                                                       # add to system
    system.refs["EE<->L2l"] = jt                                         # maintain a reference to the joint
      
    
    #no need to return, as system is passed by reference and then modified. 
    return system 

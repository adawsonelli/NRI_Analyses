#------------------------------------------------------------------------------
#        Adjustable Leaning EXersize Robot (ALEXR) Dynamic Simulations 
#------------------------------------------------------------------------------

#---------------------------- Imports -----------------------------------------
import pychrono.core as chrono
import pychrono.irrlicht as chronoirr
import numpy as np

# Change this path to asset path, if running from other working dir. 
# It must point to the data folder, containing GUI assets (textures, fonts, meshes, etc.)
chronoDataDir = "D:/programFiles/Miniconda3/pkgs/pychrono-5.0.0-py37_9/Library/data/"
chrono.SetChronoDataPath(chronoDataDir)
assetsPath = "C:/Users/adaws/Documents/gitRepos/NRI_Analyses/chrono/assets/"


# ----------- Calculate IK angles using custom Library ------------------------

## in the future, this should be formulated so that you can select a location
## Xee, Yee, and the system will initialize with all of the correct angles, etc
## to do this, we will need to import the code that we have already written, and 
## move some code from notebooks, into .py files or something to this effect,
## and then it can be directly connected to the simulation system developed below. 

#initial location of the end effector (EE) of the ALEX Robot
Xee =  .5    #(these form the initial conditions of the robot)
Yee = -.5


#set the elbow discrete vars this will flip for right vs. left side useage
side = "right"  # "left"
if side == "right":
    ef = "up"
    e3 = "down"
elif side == "left":
    ef = "down"
    e3 = "up"


#hardcode robot link lengths and origins
xl = 0 ; yl = -.27 
xr = 0 ; yr = -.055


#left robot
_L1l = .48               #meters
_L2l = .28               #meters
_L3l = .38               #meters
_L23l = _L2l + _L3l      #the angle between them is 0, therfore they may be treated as the same link


#right robot
_L1r = .28               #the right robot is a normal RR robot
_L2r = .40


#inverse kinematics equations for an RR robot
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
    

#calculate IK robot angles (this version is simplified from the Gen5bl to the case where θ=0)
θ1l , θ2l = IK_RR(Xee - xl,Yee - yl,_L1l,_L23l,ef)     #IK of the left robot to end effector location
Jx = Xee - _L3l*np.cos(θ1l + θ2l)                      #locations of the joint in 3D space 
Jy = Yee - _L3l*np.sin(θ1l + θ2l)                      #(vector subtraction method)
θ1r , θ2r = IK_RR(Jx - xr,Jy - yr,_L1r,_L2r,e3)         #IK of the right robot


#perform a check here, and throw an error if the original configuration can't be solved.
if np.isnan(θ1l and θ2l and θ1r and θ2r):
    raise ValueError("there is no solution to IK for the end-effector location specified (Xee,Yee)")

    
#define a library function for calculating FK of COM locations for each link


#--------------- Create the simulation system ---------------------------------
mysystem = chrono.ChSystemNSC()


#--------------- create each link as a rigid body -----------------------------


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
mysystem.Add(GB)


##ground left
#side = .1
#GL = chrono.ChBody()
#GL.SetBodyFixed(True)
#GL.SetPos(chrono.ChVectorD(0,yl - side/2,-0.2))
#mysystem.Add(GL)
#
#mboxasset = chrono.ChBoxShape()
#mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(side,side,0.01)
#GL.AddAsset(mboxasset)

#ground (make default object, then set visualization)

#body_A = chrono.ChBodyEasyMesh(assetsPath +'ground.obj', # mesh filename
#                               7000,                     # density kg/m^3
#                               True,                     # use mesh for visualization?
#                               False)                    # use mesh for collision?
#
#body_A.SetBodyFixed(True)
#body_A.SetPos(chrono.ChVectorD(0,0,0))
#body_A.GetAssets.
#mysystem.Add(body_A) 

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
mysystem.Add(coord)


#----------- left link 1 ------------------
#add body
L1l = chrono.ChBodyAuxRef()
L1l.SetBodyFixed(True)
mysystem.Add(L1l)

#add mass properties  //improve these based on actual data...
L1l.SetMass(1)

#set position,orientation with FK
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


#----------- left link 2 ------------------
#add body
L2l = chrono.ChBodyAuxRef()
L2l.SetBodyFixed(True)
mysystem.Add(L2l)

#add mass properties  //improve these based on actual data...
L1l.SetMass(1)

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


#----------- right link 1 -----------------
#add body
L1r = chrono.ChBodyAuxRef()
L1r.SetBodyFixed(True)
mysystem.Add(L1r)

#add mass properties  //improve these based on actual data...
L1l.SetMass(1)

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

#----------- right link 2 -----------------
#add body
L2r =  chrono.ChBodyAuxRef()
L2r.SetBodyFixed(True)
mysystem.Add(L2r)

#add mass properties  //improve these based on actual data...
L1l.SetMass(1)

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

#----------- end effector payload ---------










##link 1 left
#L1l = chrono.ChBody()
#L1l.SetBodyFixed(False)
#mysystem.Add(L1l)
#
##add mass properties here
#
#mboxasset = chrono.ChBoxShape()
#mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.1,_L1l,0.1)
#L1l.AddAsset(mboxasset)

##link 2 left
#L2l = chrono.ChBody()
#L2l.SetBodyFixed(False)
#mysystem.Add(L2l)
#
##add mass properties here 
#
#mboxasset = chrono.ChBoxShape()
#mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.1,_L2l,0.1)
#L1l.AddAsset(mboxasset)

#
##link 1 right
#L1r = chrono.ChBody()
#L1r.SetBodyFixed(False)
#mysystem.Add(L1r)
#
##add mass properties here
#L1r.SetRot(chrono.ChMatrix33D(4,chrono.ChVectorD(0,0,1)))
#
#
#mboxasset = chrono.ChBoxShape()
#mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.025,_L1r,0.01)
#L1r.AddAsset(mboxasset)


#link 2 right
#L2r = chrono.ChBody()
#L2r.SetBodyFixed(False)
#mysystem.Add(L2r)
#
##add mass properties here
#
#mboxasset = chrono.ChBoxShape()
#mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.025,_L2r,0.01)
#L1r.AddAsset(mboxasset)
#






## Create a fixed rigid body
#
#mbody1 = chrono.ChBody()
#mbody1.SetBodyFixed(True)
#mbody1.SetPos( chrono.ChVectorD(0,0,-0.2))
#mysystem.Add(mbody1)
#
#mboxasset = chrono.ChBoxShape()
#mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.2,0.5,0.1)
#mbody1.AddAsset(mboxasset)
#
#
#
## Create a swinging rigid body
#
#mbody2 = chrono.ChBody()
#mbody2.SetBodyFixed(False)
#mysystem.Add(mbody2)
#
#mboxasset = chrono.ChBoxShape()
#mboxasset.GetBoxGeometry().Size = chrono.ChVectorD(0.2,0.5,0.1)
#mbody2.AddAsset(mboxasset)
#
#mboxtexture = chrono.ChTexture()
#mboxtexture.SetTextureFilename('../../../data/concrete.jpg')
#mbody2.GetAssets().push_back(mboxtexture)


#----------------------- create the revolute joints ---------------------------


#GL <-> L1l
#
##create revolute joint object
#mlink = chrono.ChLinkRevolute()
#
## the coordinate system of the constraint reference in abs. space:
#mframe = chrono.ChFrameD(chrono.ChVectorD(0.1,0.5,0))
#
## initialize the constraint telling which part must be connected, and where:
#mlink.Initialize(GL,L1l, mframe)
#
##add to system
#mysystem.Add(mlink)


# L2l <-> L23l

# GR <-> L1r
mlink = chrono.ChLinkRevolute()                                 #create revolute joint object

local = True                                                    #we will use the local frame
GR_frame =  chrono.ChFrameD(chrono.ChVectorD(0,0,0))            #local frame of attachment
L1r_frame = chrono.ChFrameD(chrono.ChVectorD(0.0,-1*_L1r,-.02))     #local frame of attachment

mlink.Initialize(GL,L1r,local,GR_frame,L1r_frame)
mysystem.Add(mlink)

## l1r <-> L2r
#mmlink = chrono.ChLinkRevolute()                                 #create revolute joint object
#
#local = True                                                    #we will use the local frame
#L1r_frame =  chrono.ChFrameD(chrono.ChVectorD(0,-1*_L1r/2,0))        #local frame of attachment
#L2r_frame = chrono.ChFrameD(chrono.ChVectorD(0.0,0.0,-.02))     #local frame of attachment
#
#mmlink.Initialize(L1r,L2r,local,L1r_frame,L2r_frame)
#mysystem.Add(mmlink)
#


#L1r.SetRot(chrono.ChMatrix33D(.57,chrono.ChVectorD(0,0,1)))

# -------------------------- setup torque motors ------------------------------



# -----------------------------------------------------------------------------
#            Create an Irrlicht application to visualize the system
# -----------------------------------------------------------------------------


myapplication = chronoirr.ChIrrApp(mysystem, 'PyChrono example', chronoirr.dimension2du(1024,768))

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




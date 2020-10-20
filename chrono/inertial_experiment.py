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

#initial location of the end effector (EE) of the ALEX Robot
Xee =  .5       #(these form the initial conditions of the robot)
Yee = -.5

#specify mass properties of the payload at the end effector. 
eeMass = 1   


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
L1l.SetBodyFixed(False)
mysystem.Add(L1l)

#add mass properties
L1l.SetMass(.398)
#L1l.SetInertiaXX(chrono.ChVectorD(.00003,.00808,.00810)) #from solidworks

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
L2l.SetBodyFixed(False)
mysystem.Add(L2l)

#add mass properties  //improve these based on actual data...
m = .266 + .274
L2l.SetMass(m)
#L2l.SetInertiaXX(chrono.ChVectorD(.00005,.02053,.02057)) #from solidworks


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
L1r.SetBodyFixed(False)
mysystem.Add(L1r)

#add mass properties
L1r.SetMass(.236)
L1r.SetInertiaXX(chrono.ChVectorD(.00002,.00171,.00172))  #from solidworks

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
L2r = chrono.ChBodyAuxRef()
L2r.SetBodyFixed(False)
mysystem.Add(L2r)

#add mass properties  //improve these based on actual data...
L2r.SetMass(.334)
#L2r.SetInertiaXX(chrono.ChVectorD(.00003,.00475,.00478))


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
#add body
ee = chrono.ChBodyAuxRef()
ee.SetBodyFixed(False)
mysystem.Add(ee)

#add mass properties  //improve these based on actual data...
ee.SetMass(eeMass)
#ee.SetInertiaXX(chrono.ChVectorD(.00001,.00001,.00001))

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

#------------- GB  <-> L1l --------------
jt = chrono.ChLinkRevolute()                                         #create revolute joint object
local = True                                                         #we will use the local frame
GB_frame =  chrono.ChFrameD(chrono.ChVectorD(0,-1*(yr - yl)/2,0.01))  #local frame of attachment
L1l_frame = chrono.ChFrameD(chrono.ChVectorD(-1*_L1l/2,0,0))         #local frame of attachment
jt.Initialize(GB,L1l,local,GB_frame,L1l_frame)                       #init joint
mysystem.Add(jt)                                                     #add to system

##------------- L1l <-> L2l --------------
jt = chrono.ChLinkRevolute()                                         #create revolute joint object
local = True                                                         #we will use the local frame
L1l_frame = chrono.ChFrameD(chrono.ChVectorD(_L1l/2,0,0.01))          #local frame of attachment
L2l_frame = chrono.ChFrameD(chrono.ChVectorD(-1*_L23l/2,0,0))        #local frame of attachment
jt.Initialize(L1l,L2l,local,L1l_frame,L2l_frame)                     #init joint
mysystem.Add(jt)                                                     #add to system

##------------- GB  <-> L1r --------------
jt = chrono.ChLinkRevolute()                                         #create revolute joint object
local = True                                                         #we will use the local frame
GB_frame =  chrono.ChFrameD(chrono.ChVectorD(0,(yr - yl)/2,.02))     #local frame of attachment
L1r_frame = chrono.ChFrameD(chrono.ChVectorD(-1*_L1r/2,0,0))         #local frame of attachment
jt.Initialize(GB,L1r,local,GB_frame,L1r_frame)                       #init joint
mysystem.Add(jt)                                                     #add to system


##------------- L1r <-> L2r --------------
jt = chrono.ChLinkRevolute()                                         #create revolute joint object
local = True                                                         #we will use the local frame
L1r_frame = chrono.ChFrameD(chrono.ChVectorD(_L1r/2,0,.01))          #local frame of attachment
L2r_frame = chrono.ChFrameD(chrono.ChVectorD(-1*_L2r/2,0,0))        #local frame of attachment
jt.Initialize(L1r,L2r,local,L1r_frame,L2r_frame)                     #init joint
mysystem.Add(jt)                                                     #add to system

##------------- L2l <-> L2r --------------
jt = chrono.ChLinkRevolute()                                         #create revolute joint object
local = True                                                         #we will use the local frame
dj = -1*(_L23l/2 - _L2l)                                             #distance from center to joint point
L2l_frame = chrono.ChFrameD(chrono.ChVectorD(dj,0,.01))              #local frame of attachment
L2r_frame = chrono.ChFrameD(chrono.ChVectorD(_L2r/2,0,0))            #local frame of attachment
jt.Initialize(L2l,L2r,local,L2l_frame,L2r_frame)                     #init joint
mysystem.Add(jt)                                                     #add to system
#
##------------- EE <-> L2l --------------
jt = chrono.ChLinkRevolute()                                         #create revolute joint object
local = True                                                         #we will use the local frame                                          #distance from center to joint point
L2l_frame = chrono.ChFrameD(chrono.ChVectorD(_L23l/2,0,.01))         #local frame of attachment
ee_frame = chrono.ChFrameD(chrono.ChVectorD(0,0,0))                  #local frame of attachment
jt.Initialize(L2l,ee,local,L2l_frame,ee_frame)                       #init joint
mysystem.Add(jt)                                                     #add to system


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



#------------------------------- to do ----------------------------------------
"""
* finish specifying the inertial properties of the system
* to continue to debug this issue, make another file which is a double pendulum with the easy mesh importer 
* and see if it performs poorly. 
* figure out how to modularize the system, so that it can be imported into other analyses
* figure out how to get nice plots
* information regarding how long it takes to execute some of these analyses with a fixed, and variable timestep. 
* next analysis should be inverse dynamic analysis of the robotic mechanism (next week)
* the next next analysis should be forward dynamic analysis with a PID controller (two weeks)
* might want to consider including the inertia for a leg model. 


notes for considering refactor that will reduce code repetition.

realistically, we just want to encapsulate the building of the system, so that 
process doesn't need to be replicated in each analysis we want to perform. this 
could be done functionally, where there is a function with default arguments
that we could overload, and then the system object is returned, or it could be done
in an object oriented way, where we make a container object for the system, that 
manages aspects of the simulation, but that also makes it easy to access things for 
add on analyses. at present,

I'm leaning towards a libary of functions, that either
return or take system objects and modify those system objects and that are easy
to add on to. let me give an example usecase I'm thinking of. 

suppose we want to do some foward dynamic simulations, with a motor of a particular size and fxn. 
the model we use will be the same as any other, but to it, we will add (via scripting) a motor
with a torque that varies with time. if we have access to the system object, we can initialize it, 
then take it and call the add motor function, then pass that object to another function that 
will forward simulate the system, either for visualization, or to plot various variables. 

"""


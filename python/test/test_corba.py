#Importing helper class for RBPRM
from hpp.corbaserver.rbprm.rbprmfullbody import FullBody
from hpp.corbaserver.problem_solver import ProblemSolver
from hpp.gepetto import Viewer
#reference pose for hyq
from hyq_ref_pose import hyq_ref


from os import environ
ins_dir = environ['DEVEL_DIR']
db_dir = ins_dir+"/install/share/hyq-rbprm/database/hyq_"


from hpp.corbaserver import Client


packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"

#  Information to retrieve urdf and srdf files.
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""

#  This time we load the full body model of HyQ
fullBody = FullBody () 
fullBody.loadFullBodyModel(urdfName, rootJointType, meshPackageName, packageName, urdfSuffix, srdfSuffix)
fullBody.setJointBounds ("root_joint", [-2,5, -1, 1, 0.3, 4])

#  Setting a number of sample configurations used
nbSamples = 10000

ps = ProblemSolver(fullBody)
r = Viewer (ps)

q_init = hyq_ref[:]; 


r(q_init)

from hppy_ik import *

from numpy import array
from numpy.linalg import norm

import eigenpy
eigenpy.switchToNumpyArray()

__EPS = 1e-4

# load robot
packageName = "hyq_description"
meshPackageName = "hyq_description"
rootJointType = "freeflyer"
urdfName = "hyq"
urdfSuffix = ""
srdfSuffix = ""
robotName = "hyq"
hyq = IkHelper(robotName,rootJointType,packageName,urdfName,urdfSuffix,srdfSuffix)

def l2m(l):
        return array(l).reshape([-1,1])
        
def v2m(v):
        return v.reshape([-1,1])
        
def m2v(m):
        return v.flatten()
        
def m2l(m):
        return m.flatten().tolist()


from spline import bezier
import time 

waypoints = array([[1.,2.,3.],[4.,5.,6.]]).transpose()
a = bezier(waypoints, 3.)


def update():
        print project(hyq)
        q_ik = getConfig(hyq)  
        #~ setConfig(hyq, q_ik)
        q = m2l(q_ik)
        r(q)
        clearProjector(hyq)
        return q

def followtraj(ikHelper, fram_traj_list, maintain_constraints):
        init_clock = time.clock()
        timer = 0
        while(True):
                timer = time.clock() - init_clock
                finished = True
                for (frame, traj) in fram_traj_list:
                        if(traj.max() >= timer):
                                target3D(ikHelper,frame,traj(timer))
                                finished = False
                if(finished):
                        return
                else:
                        maintain_constraints
                        update()
                
                
        

q_ik = l2m(q_init)
 
setConfig(hyq,q_ik)
 
rff = FrameMarker(hyq, 'rf_foot_joint')
position = rff.pos()
rotation = rff.rot()

print  rff.pos()

targetPos = position + l2m([0.1,0.,0.])

target3D(hyq,rff,targetPos)

def update():
        print project(hyq)
        q_ik = getConfig(hyq)  
        #~ setConfig(hyq, q_ik)
        q = m2l(q_ik)
        r(q)
        clearProjector(hyq)
        return q

#~ for i in range(10):
        #~ targetPos = rff.pos() + l2m([0.001,0.,0.])
        #~ print targetPos
        #~ target3D(hyq,rff,targetPos)
        #~ update()




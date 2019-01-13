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

q_init = array([-2.0,
 0.0,
 0.6838277139631803,
 0.0,
 0.0,
 0.0,
 1.0,
 0.14279812395541294,
 0.934392553166556,
 -0.9968239786882757,
 -0.06521258938340457,
 -0.8831796268418511,
 1.150049183494211,
 -0.06927610020154493,
 0.9507443168724581,
 -0.8739975339028809,
 0.03995660287873871,
 -0.9577096766517215,
 0.9384602821326071]).reshape([-1,1])
 
setConfig(hyq,q_init)
assert(norm(getConfig(hyq) - q_init) < __EPS)
 
rff = FrameMarker(hyq, 'rf_foot_joint')
position = rff.pos()
rotation = rff.rot()

targetPos = position + l2m([0.1,0.,0.])

target3D(hyq,rff,targetPos)
assert(project(hyq))
assert(norm(targetPos -  rff.pos()) < __EPS)

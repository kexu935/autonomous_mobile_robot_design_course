#       __TRANSITIONFUNCTIONS__
#       This is the main file to execute functions of the Transition mode
#
#       Authors: 
#       Kostas Alexis (kalexis@unr.edu)

from __future__ import division
import numpy as np
from math import tan, sin, cos, atan2, fmod, acos, asin, pow, sqrt, fabs,atan
from ElementaryFunctions import max, min

TransitionSolution = { }
TransitionSolution['connection'] = np.zeros((2,4))
TransitionSolution['distance'] = 0
TransitionSolution['t_ex'] = 0

def computeTranstionConnection(point_0=None, deltaX=None, deltaY=None, deltaZ=None):
    point_1 = np.array(point_0)
    point_1[0] = point_1[0] + deltaX
    point_1[1] = point_1[1] + deltaY
    point_1[2] = point_1[2] + deltaZ
    L = np.array([point_0, point_1])
    return L

def computeTransitionConnectionDistance(deltaX=None, deltaY=None, deltaZ=None):
    dist_ = sqrt( pow(deltaX,2) + pow(deltaY,2) + pow(deltaZ,2) )
    return dist_

def computeTransitionConnectionTime(point_0=None, deltaX=None, deltaY=None, deltaZ=None, v_trans=None):
    dist_ = computeTransitionConnectionDistance(deltaX, deltaY, deltaZ)
    t_ex = dist_/v_trans
    return t_ex

def AssembleTransitionSolution(point_0=None, deltaX=None, deltaY=None, deltaZ=None, v_trans=None):
    L = computeTranstionConnection(point_0, deltaX, deltaY, deltaZ)
    dist_ = computeTransitionConnectionDistance(deltaX, deltaY, deltaZ)
    t_ex = computeTransitionConnectionTime(point_0, deltaX, deltaY, deltaZ, v_trans)
    TransitionSolution['connection'] = L
    TransitionSolution['distance'] = dist_
    TransitionSolution['t_ex'] = t_ex
    return TransitionSolution
    

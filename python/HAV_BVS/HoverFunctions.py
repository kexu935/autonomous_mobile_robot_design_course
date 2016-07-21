#       __HOVERFUNCTIONS__
#       This is the main file to execute functions of the Hover mode
#
#       Authors: 
#       Kostas Alexis (kalexis@unr.edu)


from __future__ import division
import numpy as np
from math import tan, sin, cos, atan2, fmod, acos, asin, pow, sqrt, fabs,atan
from ElementaryFunctions import max, min

pi = np.pi

HoverSolution = { }
HoverSolution['connection'] = np.zeros((2,4));
HoverSolution['distance'] = 0;
HoverSolution['t_ex'] = 0;

def computeHoverConnection(point_0=None, point_1=None):
    L = np.array([point_0, point_1])
    return L

def computeHoverConnectionDistance(point_0=None, point_1=None):
    dist_ = sqrt( pow(point_1[0]-point_0[0],2) + pow(point_1[1]-point_0[1],2) + pow(point_1[2]-point_0[2],2) ) 
    return dist_

def computeHoverConnectionTime(point_0=None,point_1=None,v_max=None,psi_rate_max=None):
    dist_ = computeHoverConnectionDistance(point_0, point_1)
    t_ex = max( dist_/v_max, fabs(point_1[3]-point_0[3])/psi_rate_max)
    return t_ex

def AssembleHoverSolution(point_0=None,point_1=None,v_max=None,psi_rate_max=None):
    L = computeHoverConnection(point_0, point_1)
    dist_ = computeHoverConnectionDistance(point_0, point_1)
    t_ex = computeHoverConnectionTime(point_0, point_1, v_max, psi_rate_max)
    HoverSolution['connection'] = L
    HoverSolution['distance'] = dist_
    HoverSolution['t_ex'] = t_ex
    return HoverSolution
    


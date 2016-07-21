#       __BVSCAVSTATEMACHINE__
#       This is the main file to run the state machine
#       of the convertible aerial vehicle boundary value solver
#
#       Authors: 
#       Kostas Alexis (kalexis@unr.edu)

from __future__ import division
import numpy as np

# mode_transitions_case1 = [CURRENT,INITIAL_CONVERSION,NAVIGATION,FINAL_CONVERSION]
def computeTranstionConnection(current_mode=None, end_mode=None):
    if current_mode == 0:
        if end_mode == 0:
            mode_transitions_case1 = np.array([0,0,0,0])
            mode_transitions_case2 = np.array([0,1,1,0])
        if end_mode == 1:
            mode_transitions_case1 = np.array([0,0,0,1])
            mode_transitions_case2 = np.array([0,1,1,1])
    if current_mode == 1:
        if end_mode == 0:
            mode_transitions_case1 = np.array([1,0,0,0])
            mode_transitions_case2 = np.array([1,1,1,0])
        if end_mode == 1:
            mode_transitions_case1 = np.array([1,0,0,1])
            mode_transitions_case2 = np.array([1,1,1,1])
            
    
    mode_transitions = np.array([mode_transitions_case1, mode_transitions_case2])
    return mode_transitions

            
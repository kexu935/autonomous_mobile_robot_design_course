#       __TRANSITIONMODEMAIN__
#       This is the main file to execute examples of the Transition mode
#
#       Authors: 
#       Kostas Alexis (kalexis@unr.edu)

from TransitionFunctions import * 
from PlottingTools import plot3
import numpy as np
import time
import sys

pi = np.pi
verbose_flag = 0
plot_flag = 1

point_0 = np.array([0,0,0,0])
deltaX = 0.1
deltaY = 0.1
deltaZ = 0.1

class ExecutionFlags(object):
    """
        Execution flags
    """  
    def __init__(self, verbose_flag, plot_flag):
        self.verbose = verbose_flag
        self.plot = plot_flag

class VehicleParameters(object):
    """
        Vehicle Parameters
    """ 
    def __init__(self, v_trans):
        self.v_trans = v_trans
    
def main():
    # Example main for the Hover mode
    t0 = time.clock()
    VehiclePars = VehicleParameters( 4) 
    ExFlags = ExecutionFlags( verbose_flag,plot_flag )
    
    flag_nc = 0
    fname = 'transition_solution.txt'
    TransitionSolution = AssembleTransitionSolution(point_0, deltaX, deltaY, deltaZ, VehiclePars.v_trans)
    path_trans = TransitionSolution['connection']
    np.savetxt( fname, path_trans, delimiter = ',' ) 
    if ExFlags.plot :
            print '### Transition solution plot'
            plot3( path_trans[:,0], path_trans[:,1], path_trans[:,2], 'o', 'g' )
  

def testAllCases():
    main()
    print 'Press any button to continue'
    raw_input()
        

if __name__ == "__main__":
    testAllCases()

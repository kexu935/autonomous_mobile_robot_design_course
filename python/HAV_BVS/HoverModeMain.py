#       __HOVERMODEMAIN__
#       This is the main file to execute examples of the Hover mode
#
#       Authors: 
#       Kostas Alexis (kalexis@unr.edu)

from HoverFunctions import * 
from PlottingTools import plot3
import numpy as np
import time
import sys

pi = np.pi
verbose_flag = 0
plot_flag = 1

point_0 = np.array([0,0,0,0])
point_1 = np.array([10,10,10,pi/4])

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
    def __init__(self, v_max, psi_rate_max):
        self.v_max = v_max
        self.psi_rate_max = psi_rate_max
    
def main():
    # Example main for the Hover mode
    t0 = time.clock()
    VehiclePars = VehicleParameters( 4, pi/4,) 
    ExFlags = ExecutionFlags( verbose_flag,plot_flag )
    
    flag_nc = 0
    fname = 'hover_solution.txt'
    HoverSolution = AssembleHoverSolution(point_0, point_1, VehiclePars.v_max, VehiclePars.psi_rate_max)
    path_hover = HoverSolution['connection']
    np.savetxt( fname, path_hover, delimiter = ',' ) 
    if ExFlags.plot :
            print '### Hover solution plot'
            plot3( path_hover[:,0], path_hover[:,1], path_hover[:,2], 'o', 'g' )
  

def testAllCases():
    main()
    print 'Press any button to continue'
    raw_input()
        

if __name__ == "__main__":
    testAllCases()

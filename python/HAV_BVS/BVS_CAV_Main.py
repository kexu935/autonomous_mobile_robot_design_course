#       __BVSCAVMAIN__
#       This is the main file to execute examples of the
#       Boundary Value Solver for Convertible Aerial Vehicles
#
#       Authors: 
#       Kostas Alexis (kalexis@unr.edu)

from bvs_cav_statemachine import *
from bvs_cav_ModeFusionFunctions import *
from PlottingTools import plot3
import numpy as np
import time
import sys

# define auxiliary variables
pi = np.pi
verbose_flag = 0
plot_flag = 1

# define cav current and end mode
current_mode = 0
end_mode = 0   

# define starting and final configuration
point_0 = np.array([0,0,50,0])
point_1 = np.array([50,200,200,0])

# name the file to save to
fname = 'cav_solution.txt'

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
    def __init__(self, Vairspeed_0, Bank_max, Gamma_max, v_max, psi_rate_max, v_trans,deltaXhf,deltaYhf,deltaZhf,deltaXfh,deltaYfh,deltaZfh):
        self.Vairspeed_0 = Vairspeed_0 # airspeed of the vehicle in fixed-wing mode
        self.Bank_max = Bank_max # maximum bank angle of the vehicle in fixed-wing mode
        self.Gamma_max = Gamma_max # gamma maximum angle of the path of the vehicle in fixed-wing mode
        self.v_max = v_max # maximum velocity in rotorcraft mode
        self.psi_rate_max = psi_rate_max # maximum yaw rate in rotorcraft mode
        self.v_trans = v_trans # transition velocity
        self.deltaXhf = deltaXhf # delta X for hover to forward flight
        self.deltaYhf = deltaYhf # delta Y for hover to forward flight
        self.deltaZhf = deltaZhf # delta Z for hover to forward flight
        self.deltaXfh = deltaXfh # delta X for forward to hover flight
        self.deltaYfh = deltaYfh # delta Y for forward to hover flight
        self.deltaZfh = deltaZfh # delta Z for forward to hover flight
        
def main():
    VehiclePars = VehicleParameters( 15, pi/4, pi/6, 4, pi/4, 5,2,2,2,5,5,-5) 
    ExFlags = ExecutionFlags( verbose_flag,plot_flag )
    mode_transitions = computeTranstionConnection(current_mode, end_mode)

    CAVSolution = computeTransitionPath(current_mode, end_mode, point_0, point_1, VehiclePars)
    np.savetxt( fname, CAVSolution['total_connection'], delimiter = ',' ) 
    print '### Covertible Aerial Vehicle solution saved in ' + fname
    if ExFlags.plot :
        print '### Covertible Aerial Vehicle solution plot'
        plot3( CAVSolution['total_connection'][:,0], CAVSolution['total_connection'][:,1], CAVSolution['total_connection'][:,2], 'o', 'g' )
    

def RunExample():
    main()
    print 'Press any button to continue'
    raw_input()
        

if __name__ == "__main__":
    RunExample()
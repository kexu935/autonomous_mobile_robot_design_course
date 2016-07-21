#       __BVSCAVMODELFUSIONFUNCTIONS__
#       This is the main file to run the model 
#       fusion functions for the boundary value solver
#       of convertible aerial vehicles
#
#       Authors: 
#       Kostas Alexis (kalexis@unr.edu)

from __future__ import division
import numpy as np
from math import tan, sin, cos, atan2, fmod, acos, asin, pow, sqrt, fabs,atan
from ElementaryFunctions import max, min

from DubinsAirplaneFunctions import * 
from HoverFunctions import *
from TransitionFunctions import *
from bvs_cav_statemachine import *

CAVSolution = { }
CAVSolution['connection'] = { }
CAVSolution['distance'] = 0
CAVSolution['t_ex'] = 0
CAVSolution['class'] = 'HHH'

BestSolution = { }
BestSolution['SEG1_connection'] = { }
BestSolution['SEG1_distance'] = { }
BestSolution['SEG1_t_ex'] = { }
BestSolution['SEG2_connection'] = { }
BestSolution['SEG2_distance'] = { }
BestSolution['SEG2_t_ex'] = { }
BestSolution['SEG3_connection'] = { }
BestSolution['SEG3_distance'] = { }
BestSolution['SEG3_t_ex'] = { }
BestSolution['class'] = { }
BestSolution['total_connection'] = { }
BestSolution['total_distance'] = 0
BestSolution['total_t_ex'] = 0

InitConversionSolution = { }
InitConversionSolution['connection'] = { }
InitConversionSolution['distance'] = { }
InitConversionSolution['t_ex'] = { }
NaviSolution ={ }
NaviSolution['connection'] = { }
NaviSolution['distance'] = { }
NaviSolution['t_ex'] = { }
FinalConversionSolution ={ }
FinalConversionSolution['connection'] = { }
FinalConversionSolution['distance'] = { }
FinalConversionSolution['t_ex'] = { }

# mode_transitions_case1 = [CURRENT,INITIAL_CONVERSION,NAVIGATION,FINAL_CONVERSION]

def computeTransitionPath(current_mode=None, end_mode=None, point_0=None, point_1=None, VehiclePars=None):
    R_min = MinTurnRadius_DubinsAirplane( VehiclePars.Vairspeed_0, VehiclePars.Bank_max )
    mode_transitions_all = computeTranstionConnection(current_mode, end_mode)
    
    best_path_cost = pow(10,9)
    for i in range(0,2):
        
        mode_transitions = mode_transitions_all[i][:]
        if mode_transitions[0] == 0 and mode_transitions[1] == 0 and mode_transitions[2] == 0 and mode_transitions[3] == 0:
            # TRANSITION: HHH
            # hover-hover
            InitConversionSolution['connection'] = np.array([point_0,point_0])
            InitConversionSolution['distance'] = 0
            InitConversionSolution['t_ex'] = 0
            # hover - navi @ hover
            point_start = InitConversionSolution['connection'][-1]
            NaviSolution = AssembleHoverSolution(point_start, point_1, VehiclePars.v_max, VehiclePars.psi_rate_max)
            path_navi = NaviSolution['connection']
            # navi hover -> hover
            point_end = NaviSolution['connection'][-1]
            FinalConversionSolution['connection'] = np.array([point_end,point_1])
            FinalConversionSolution['distance'] = 0
            FinalConversionSolution['t_ex'] = 0
            transition_class = 'HHH'
            path_cost = InitConversionSolution['t_ex'] + NaviSolution['t_ex'] + FinalConversionSolution['t_ex']
            if path_cost < best_path_cost: 
                BestSolution['SEG1_connection'] = InitConversionSolution['connection']
                BestSolution['SEG1_distance'] = InitConversionSolution['distance']
                BestSolution['SEG1_t_ex'] = InitConversionSolution['t_ex']
                BestSolution['SEG2_connection'] = NaviSolution['connection']
                BestSolution['SEG2_distance'] = NaviSolution['distance']
                BestSolution['SEG2_t_ex'] = NaviSolution['t_ex']
                BestSolution['SEG3_connection'] = FinalConversionSolution['connection']
                BestSolution['SEG3_distance'] = FinalConversionSolution['distance']
                BestSolution['SEG3_t_ex'] = FinalConversionSolution['t_ex']
                BestSolution['class'] = transition_class
                best_path_cost = path_cost
                
            
        if mode_transitions[0] == 0 and mode_transitions[1] == 1 and mode_transitions[2] == 1 and mode_transitions[3] == 0:
            # TRANSITION: HFH
            # hover-froward
            TransitionSolution = AssembleTransitionSolution(point_0, VehiclePars.deltaXhf, VehiclePars.deltaYhf, VehiclePars.deltaZhf, VehiclePars.v_trans)
            InitConversionSolution['connection'] = TransitionSolution['connection']
            InitConversionSolution['distance'] = TransitionSolution['distance']
            InitConversionSolution['t_ex'] = TransitionSolution['t_ex']
            # forward - navi forward
            point_start = InitConversionSolution['connection'][-1]
            DubinsAirplaneSolution = DubinsAirplanePath( point_start, point_1, R_min, VehiclePars.Gamma_max )
            path_dubins_airplane = ExtractDubinsAirplanePath( DubinsAirplaneSolution )
            path_dubins_airplane = path_dubins_airplane.T
            size_dubins_path = path_dubins_airplane.shape
            zeros_col = np.zeros((size_dubins_path[0],1))
            path_dubins_airplane = np.hstack((path_dubins_airplane,zeros_col))
            NaviSolution['connection'] = path_dubins_airplane
            NaviSolution['dist'] = DubinsPathDistance(path_dubins_airplane)
            NaviSolution['t_ex'] = NaviSolution['dist']/VehiclePars.Vairspeed_0
            # forward-hover
            point_end = NaviSolution['connection'][-1]
            ConversionSolution = AssembleTransitionSolution(point_end, VehiclePars.deltaXfh, VehiclePars.deltaYfh, VehiclePars.deltaZfh, VehiclePars.v_trans)
            FinalConversionSolution['connection'] = ConversionSolution['connection']
            FinalConversionSolution['distance'] = ConversionSolution['distance']
            FinalConversionSolution['t_ex'] = ConversionSolution['t_ex']
            transition_class = 'HFH'
            path_cost = InitConversionSolution['t_ex'] + NaviSolution['t_ex'] + FinalConversionSolution['t_ex']
            if path_cost < best_path_cost: 
                BestSolution['SEG1_connection'] = InitConversionSolution['connection']
                BestSolution['SEG1_distance'] = InitConversionSolution['distance']
                BestSolution['SEG1_t_ex'] = InitConversionSolution['t_ex']
                BestSolution['SEG2_connection'] = NaviSolution['connection']
                BestSolution['SEG2_distance'] = NaviSolution['distance']
                BestSolution['SEG2_t_ex'] = NaviSolution['t_ex']
                BestSolution['SEG3_connection'] = FinalConversionSolution['connection']
                BestSolution['SEG3_distance'] = FinalConversionSolution['distance']
                BestSolution['SEG3_t_ex'] = FinalConversionSolution['t_ex']
                BestSolution['class'] = transition_class
                best_path_cost = path_cost
            
        if mode_transitions[0] == 0 and mode_transitions[1] == 0 and mode_transitions[2] == 0 and mode_transitions[3] == 1:
            # TRANSITION: HHF
            # hover-hover
            InitConversionSolution['connection'] = np.array([point_0,point_0])
            InitConversionSolution['distance'] = 0
            InitConversionSolution['t_ex'] = 0
            # hover navi
            point_start = InitConversionSolution['connection'][-1]
            NaviSolution = AssembleHoverSolution(point_start, point_1, VehiclePars.v_max, VehiclePars.psi_rate_max)
            path_navi = NaviSolution['connection']
            # hover-froward
            point_end = NaviSolution['connection'][-1]
            ConversionSolution = AssembleTransitionSolution(point_end, VehiclePars.deltaXhf, VehiclePars.deltaYhf, VehiclePars.deltaZhf, VehiclePars.v_trans)
            FinalConversionSolution['connection'] = ConversionSolution['connection']
            FinalConversionSolution['distance'] = ConversionSolution['distance']
            FinalConversionSolution['t_ex'] = ConversionSolution['t_ex']
            transition_class = 'HHF'
            path_cost = InitConversionSolution['t_ex'] + NaviSolution['t_ex'] + FinalConversionSolution['t_ex']
            if path_cost < best_path_cost: 
                BestSolution['SEG1_connection'] = InitConversionSolution['connection']
                BestSolution['SEG1_distance'] = InitConversionSolution['distance']
                BestSolution['SEG1_t_ex'] = InitConversionSolution['t_ex']
                BestSolution['SEG2_connection'] = NaviSolution['connection']
                BestSolution['SEG2_distance'] = NaviSolution['distance']
                BestSolution['SEG2_t_ex'] = NaviSolution['t_ex']
                BestSolution['SEG3_connection'] = FinalConversionSolution['connection']
                BestSolution['SEG3_distance'] = FinalConversionSolution['distance']
                BestSolution['SEG3_t_ex'] = FinalConversionSolution['t_ex']
                BestSolution['class'] = transition_class
                best_path_cost = path_cost
            
        if mode_transitions[0] == 0 and mode_transitions[1] == 1 and mode_transitions[2] == 1 and mode_transitions[3] == 1:
            # TRANSITION: HFF
            # hover-forward
            TransitionSolution = AssembleTransitionSolution(point_0, VehiclePars.deltaXhf, VehiclePars.deltaYhf, VehiclePars.deltaZhf, VehiclePars.v_trans)
            InitConversionSolution['connection'] = TransitionSolution['connection']
            InitConversionSolution['distance'] = TransitionSolution['distance']
            InitConversionSolution['t_ex'] = TransitionSolution['t_ex']
            # forward - navi forward
            point_start = InitConversionSolution['connection'][-1]
            DubinsAirplaneSolution = DubinsAirplanePath( point_start, point_1, R_min, VehiclePars.Gamma_max )
            path_dubins_airplane = ExtractDubinsAirplanePath( DubinsAirplaneSolution )
            path_dubins_airplane = path_dubins_airplane.T
            size_dubins_path = path_dubins_airplane.shape
            zeros_col = np.zeros((size_dubins_path[0],1))
            path_dubins_airplane = np.hstack((path_dubins_airplane,zeros_col))
            NaviSolution['connection'] = path_dubins_airplane
            NaviSolution['dist'] = DubinsPathDistance(path_dubins_airplane)
            NaviSolution['t_ex'] = NaviSolution['dist']/VehiclePars.Vairspeed_0
            # forward-forward
            point_end = NaviSolution['connection'][-1]
            FinalConversionSolution['connection'] = np.array([point_end,point_1])
            FinalConversionSolution['distance'] = 0
            FinalConversionSolution['t_ex'] = 0
            transition_class = 'HFF'
            path_cost = InitConversionSolution['t_ex'] + NaviSolution['t_ex'] + FinalConversionSolution['t_ex']
            if path_cost < best_path_cost: 
                BestSolution['SEG1_connection'] = InitConversionSolution['connection']
                BestSolution['SEG1_distance'] = InitConversionSolution['distance']
                BestSolution['SEG1_t_ex'] = InitConversionSolution['t_ex']
                BestSolution['SEG2_connection'] = NaviSolution['connection']
                BestSolution['SEG2_distance'] = NaviSolution['distance']
                BestSolution['SEG2_t_ex'] = NaviSolution['t_ex']
                BestSolution['SEG3_connection'] = FinalConversionSolution['connection']
                BestSolution['SEG3_distance'] = FinalConversionSolution['distance']
                BestSolution['SEG3_t_ex'] = FinalConversionSolution['t_ex']
                BestSolution['class'] = transition_class
                best_path_cost = path_cost
            
        if mode_transitions[0] == 1 and mode_transitions[1] == 0 and mode_transitions[2] == 0 and mode_transitions[3] == 0:
            # TRANSITION: FHH
            # forward - hover
            TransitionSolution = AssembleTransitionSolution(point_0, VehiclePars.deltaXfh, VehiclePars.deltaYfh, VehiclePars.deltaZfh, VehiclePars.v_trans)
            InitConversionSolution['connection'] = TransitionSolution['connection']
            InitConversionSolution['distance'] = TransitionSolution['distance']
            InitConversionSolution['t_ex'] = TransitionSolution['t_ex']
            # hover navi
            point_start = InitConversionSolution['connection'][-1]
            NaviSolution = AssembleHoverSolution(point_start, point_1, VehiclePars.v_max, VehiclePars.psi_rate_max)
            path_navi = NaviSolution['connection']
            # hover - hover
            point_end = NaviSolution['connection'][-1]
            FinalConversionSolution['connection'] = np.array([point_end,point_1])
            FinalConversionSolution['distance'] = 0
            FinalConversionSolution['t_ex'] = 0
            transition_class = 'FHH'
            path_cost = InitConversionSolution['t_ex'] + NaviSolution['t_ex'] + FinalConversionSolution['t_ex']
            if path_cost < best_path_cost: 
                BestSolution['SEG1_connection'] = InitConversionSolution['connection']
                BestSolution['SEG1_distance'] = InitConversionSolution['distance']
                BestSolution['SEG1_t_ex'] = InitConversionSolution['t_ex']
                BestSolution['SEG2_connection'] = NaviSolution['connection']
                BestSolution['SEG2_distance'] = NaviSolution['distance']
                BestSolution['SEG2_t_ex'] = NaviSolution['t_ex']
                BestSolution['SEG3_connection'] = FinalConversionSolution['connection']
                BestSolution['SEG3_distance'] = FinalConversionSolution['distance']
                BestSolution['SEG3_t_ex'] = FinalConversionSolution['t_ex']
                BestSolution['class'] = transition_class
                best_path_cost = path_cost
            
        if mode_transitions[0] == 1 and mode_transitions[1] == 1 and mode_transitions[2] == 1 and mode_transitions[3] == 0:
            # TRANSITION: FFH
            # forward-forward
            InitConversionSolution['connection'] = np.array([point_0,point_0])
            InitConversionSolution['distance'] = 0
            InitConversionSolution['t_ex'] = 0
            # forward navi
            point_start = InitConversionSolution['connection'][-1]
            DubinsAirplaneSolution = DubinsAirplanePath( point_start, point_1, R_min, VehiclePars.Gamma_max )
            path_dubins_airplane = ExtractDubinsAirplanePath( DubinsAirplaneSolution )
            path_dubins_airplane = path_dubins_airplane.T
            size_dubins_path = path_dubins_airplane.shape
            zeros_col = np.zeros((size_dubins_path[0],1))
            path_dubins_airplane = np.hstack((path_dubins_airplane,zeros_col))
            NaviSolution['connection'] = path_dubins_airplane
            NaviSolution['dist'] = DubinsPathDistance(path_dubins_airplane)
            NaviSolution['t_ex'] = NaviSolution['dist']/VehiclePars.Vairspeed_0
            # forward-hover
            ConversionSolution = AssembleTransitionSolution(point_1, VehiclePars.deltaXfh, VehiclePars.deltaYfh, VehiclePars.deltaZfh, VehiclePars.v_trans)
            FinalConversionSolution['connection'] = ConversionSolution['connection']
            FinalConversionSolution['distance'] = ConversionSolution['distance']
            FinalConversionSolution['t_ex'] = ConversionSolution['t_ex']
            transition_class = 'FFH'
            path_cost = InitConversionSolution['t_ex'] + NaviSolution['t_ex'] + FinalConversionSolution['t_ex']
            if path_cost < best_path_cost: 
                BestSolution['SEG1_connection'] = InitConversionSolution['connection']
                BestSolution['SEG1_distance'] = InitConversionSolution['distance']
                BestSolution['SEG1_t_ex'] = InitConversionSolution['t_ex']
                BestSolution['SEG2_connection'] = NaviSolution['connection']
                BestSolution['SEG2_distance'] = NaviSolution['distance']
                BestSolution['SEG2_t_ex'] = NaviSolution['t_ex']
                BestSolution['SEG3_connection'] = FinalConversionSolution['connection']
                BestSolution['SEG3_distance'] = FinalConversionSolution['distance']
                BestSolution['SEG3_t_ex'] = FinalConversionSolution['t_ex']
                BestSolution['class'] = transition_class
                best_path_cost = path_cost
            
        if mode_transitions[0] == 1 and mode_transitions[1] == 0 and mode_transitions[2] == 0 and mode_transitions[3] == 1:
            # forward-hover
            TransitionSolution = AssembleTransitionSolution(point_0, VehiclePars.deltaXfh, VehiclePars.deltaYfh, VehiclePars.deltaZfh, VehiclePars.v_trans)
            InitConversionSolution['connection'] = TransitionSolution['connection']
            InitConversionSolution['distance'] = TransitionSolution['distance']
            InitConversionSolution['t_ex'] = TransitionSolution['t_ex']
            # hover navi
            point_start = InitConversionSolution['connection'][-1]
            NaviSolution = AssembleHoverSolution(point_start, point_1, VehiclePars.v_max, VehiclePars.psi_rate_max)
            path_navi = NaviSolution['connection']
            # hover - forward
            ConversionSolution = AssembleTransitionSolution(point_1, VehiclePars.deltaXhf, VehiclePars.deltaYhf, VehiclePars.deltaZhf, VehiclePars.v_trans)
            FinalConversionSolution['connection'] = ConversionSolution['connection']
            FinalConversionSolution['distance'] = ConversionSolution['distance']
            FinalConversionSolution['t_ex'] = ConversionSolution['t_ex']
            transition_class = 'FHF'
            path_cost = InitConversionSolution['t_ex'] + NaviSolution['t_ex'] + FinalConversionSolution['t_ex']
            if path_cost < best_path_cost: 
                BestSolution['SEG1_connection'] = InitConversionSolution['connection']
                BestSolution['SEG1_distance'] = InitConversionSolution['distance']
                BestSolution['SEG1_t_ex'] = InitConversionSolution['t_ex']
                BestSolution['SEG2_connection'] = NaviSolution['connection']
                BestSolution['SEG2_distance'] = NaviSolution['distance']
                BestSolution['SEG2_t_ex'] = NaviSolution['t_ex']
                BestSolution['SEG3_connection'] = FinalConversionSolution['connection']
                BestSolution['SEG3_distance'] = FinalConversionSolution['distance']
                BestSolution['SEG3_t_ex'] = FinalConversionSolution['t_ex']
                BestSolution['class'] = transition_class
                best_path_cost = path_cost
            
        if mode_transitions[0] == 1 and mode_transitions[1] == 1 and mode_transitions[2] == 1 and mode_transitions[3] == 1:
            # forward-forward
            InitConversionSolution['connection'] = np.array([point_0,point_0])
            InitConversionSolution['distance'] = 0
            InitConversionSolution['t_ex'] = 0
            # forward navi
            point_start = InitConversionSolution['connection'][-1]
            DubinsAirplaneSolution = DubinsAirplanePath( point_start, point_1, R_min, VehiclePars.Gamma_max )
            path_dubins_airplane = ExtractDubinsAirplanePath( DubinsAirplaneSolution )
            path_dubins_airplane = path_dubins_airplane.T
            size_dubins_path = path_dubins_airplane.shape
            zeros_col = np.zeros((size_dubins_path[0],1))
            path_dubins_airplane = np.hstack((path_dubins_airplane,zeros_col))
            NaviSolution['connection'] = path_dubins_airplane
            NaviSolution['dist'] = DubinsPathDistance(path_dubins_airplane)
            NaviSolution['t_ex'] = NaviSolution['dist']/VehiclePars.Vairspeed_0
            # forward-forward
            point_end = NaviSolution['connection'][-1]
            FinalConversionSolution['connection'] = np.array([point_end,point_1])
            FinalConversionSolution['distance'] = 0
            FinalConversionSolution['t_ex'] = 0
            transition_class = 'FFF'
            path_cost = InitConversionSolution['t_ex'] + NaviSolution['t_ex'] + FinalConversionSolution['t_ex']
            if path_cost < best_path_cost: 
                BestSolution['SEG1_connection'] = InitConversionSolution['connection']
                BestSolution['SEG1_distance'] = InitConversionSolution['distance']
                BestSolution['SEG1_t_ex'] = InitConversionSolution['t_ex']
                BestSolution['SEG2_connection'] = NaviSolution['connection']
                BestSolution['SEG2_distance'] = NaviSolution['distance']
                BestSolution['SEG2_t_ex'] = NaviSolution['t_ex']
                BestSolution['SEG3_connection'] = FinalConversionSolution['connection']
                BestSolution['SEG3_distance'] = FinalConversionSolution['distance']
                BestSolution['SEG3_t_ex'] = FinalConversionSolution['t_ex']
                BestSolution['class'] = transition_class
                best_path_cost = path_cost
                
    
    
    BestSolution['all_segs'] = np.array([BestSolution['SEG1_connection'],BestSolution['SEG2_connection'],BestSolution['SEG3_connection']])
    BestSolution['total_connection'] = np.vstack((BestSolution['SEG1_connection'],BestSolution['SEG2_connection'],BestSolution['SEG3_connection']))
    BestSolution['total_distance'] = BestSolution['SEG1_distance'] + BestSolution['SEG2_distance'] + BestSolution['SEG3_distance']
    BestSolution['total_t_ex'] = BestSolution['SEG1_t_ex'] + BestSolution['SEG2_t_ex'] + BestSolution['SEG3_t_ex']
    print BestSolution['total_connection']
    print BestSolution['total_distance']
    print BestSolution['total_t_ex'] 

    return BestSolution


import sys
sys.path.insert(0, '../build') #this is cursed

from hopper import *
import hopper
import time
import threading
import numpy as np
import networkx as networkx
import multiprocessing
import ctypes

import random as rand

from utils import *

##############################################################################################

def runSimulation(x0, xg, terminate_cond, sim_type, visualize):
    """ Run a forward simulation of the hopper given an initial condition, goal state, and termination condition
    Parameters:
        x0 (list): initial state as 21 dimensional list
        xg (list): goal state as 21 dimensional list
        terminate_cond (float): either a maximum time horizon or the maximum number of apex-to-apex hops
        sim_type (char): 'T' for max time horizon or 'H' for maximum number of hops
        visualize (bool): True to visualize the simulation
    Returns: 
        xf (list): final state as 21 dimensional list
        objVal (float): objective value
    """

    # create controller and sim objects
    c = hopper.Controller()
    s = hopper.Simulator()

    # instantiate thread for controller and sim
    control_thread = threading.Thread(target=call_run, args=(c,))
    sim_thread = threading.Thread(target=call_run_sim, args=(s,))

    # set simulator visualization
    s.setVisualization(visualize)

    # set inital state and goal state, ( pos[3], rpy[3], v[3], omega[3] )
    c.num_hops = 0
    c.setInitialState(x0)
    c.setGoalState(xg)

    # start control and sim threads
    control_thread.start()
    sim_thread.start()

    # run until termination condition
    if sim_type == 'T':
        sim_duration = terminate_cond  # time in seconds before sim stops
        while c.t_last < sim_duration:
            pass

    elif sim_type == 'H':
        max_hops = terminate_cond      # maximum number of hops before sim stops
        while c.num_hops < max_hops:
            pass
        
    else:
        print("choose either \'T\' or \'H\' for \'sim_type\'")
        quit()

    # stop simulation
    c.stopSimulation() 

    # get final state and objective value
    xf = c.x
    objVal = c.objVal

    # safely shutdown the threads
    control_thread.join()
    sim_thread.join()

    return xf, objVal

def validationSimulation(x0, waypts, parameterization, sim_type, visualize):
    """ Run a simulation to validate sequence of waypoints
    Parameters:
        x0 (list): initial state as 21 dimensional list
        waypts (list of lists): list of 21 dimensional lists (state waypoints)
        parameterization (list): list of parameterization values (time intervals or number of hops)
        sim_type (char): 'T' for time based parameterization or 'H' for hops based parameterization
        visualize (bool): True to visualize the simulation
    Returns: 
        xf (list): final state as 21 dimensional list
        objVal (float): objective value based on last waypoint
    """

    # create controller and sim objects
    c = hopper.Controller()
    s = hopper.Simulator()

    # instantiate thread for controller and sim
    control_thread = threading.Thread(target=call_run, args=(c,))
    sim_thread = threading.Thread(target=call_run_sim, args=(s,))

    # set simulator visualization
    s.setVisualization(visualize)

    # set inital state and goal state, ( pos[3], rpy[3], v[3], omega[3] )
    c.num_hops = 0
    c.setSimType(sim_type)
    c.setInitialState(x0)
    c.setStateSequence(waypts,parameterization)
    c.setSimType(sim_type)

    # start control and sim threads
    control_thread.start()
    sim_thread.start()

    # run until termination condition
    if sim_type == 'T':
        sim_duration = parameterization[-1]  # time in seconds before sim stops
        while c.t_last < sim_duration:
            pass

    elif sim_type == 'H':
        max_hops = parameterization[-1]      # maximum number of hops before sim stops
        while c.num_hops < max_hops:
            pass
        
    else:
        print("choose either \'T\' or \'H\' for \'sim_type\'")
        quit()

    # stop simulation
    c.stopSimulation() 

    # get final state and objective value
    xf = c.x
    objVal = c.objVal

    # safely shutdown the threads
    control_thread.join()
    sim_thread.join()

    return xf, objVal

##############################################################################################
# state information
# [11] hopper.q = x, y, z, qx, qy, qz, qw, L, fw1_pos, fw2_pos, fw3_pos
# [10] hopper.v = xdot, ydot, zdot, omega_x, omega_y, omega_z, Ldot, fw1_vel, fw2_vel, fw3_vel
# [21] x = [hopper.q; hopper.v]

# Sample Space [x,y,z,r,p,y, x_dot, y_dot, z_dot, omega_x, omega_y, omega_z]
# x_s = [x, y, 0.5, 0, 0, 0, xdot, ydot, 0, 0, 0] <--- sample like this
##############################################################################################

# rand.seed(3)
# # test drive the simulation
# for i in range(10):

#     print("*" * 80)
#     print("Test Drive: ", i+1)

#     # # set init conditions
#     # pos =   [2*rand.random()-1, 2*rand.random()-1, 0.5]
#     # vec = [rand.random()-0.5,rand.random()-0.5,rand.random()-0.5,rand.random()-0.5] # [qx, qy, qz, qw] MUST normalize
#     # quat = vec / np.linalg.norm(vec)
#     # vel =   [1*rand.random()-0.5, 1*rand.random()-0.5, 1*rand.random()-0.5]
#     # omega = [0.1*rand.random()-0.05, 0.1*rand.random()-0.05, 0.1*rand.random()-0.05]

#     # x0 = [pos[0], pos[1], pos[2],              # pos[3]
#     #       quat[0], quat[1], quat[2], quat[3],  # quat[4] , [qx, qy, qz, qw] MUST be unit
#     #       0.05,                                # L[1] (0.05 nominal)                 
#     #       0,0,0,                               # flywheel[3], setting this does nothing
#     #       vel[0], vel[1], vel[2],              # v[3]
#     #       omega[0], omega[1], omega[2],        # omega[3]
#     #       0,                                   # Ldot[1]                    
#     #       0,0,0]                               # flywheel_dot[3]
#    # set goal state
#     x0 = [1,1,0.5,  # pos[3]
#           0,0,0,1,  # quat[4] , [qx, qy, qz, qw], MUST normalize
#           .05,      # L[1] (0.05 nominal)
#           0,0,0,    # flywheel[3], setting this does nothing
#           0,0,0,    # v[3]
#           0,0,0,    # omega[3]
#           0,        # Ldot[1]
#           0,0,0]    # flywheel_dot[3]

#     # set goal state
#     xg = [0,0,0.5,  # pos[3]
#           0,0,0,1,  # quat[4] , [qx, qy, qz, qw], MUST normalize
#           .05,      # L[1] (0.05 nominal)
#           0,0,0,    # flywheel[3], setting this does nothing
#           0,0,0,    # v[3]
#           0,0,0,    # omega[3]
#           0,        # Ldot[1]
#           0,0,0]    # flywheel_dot[3]
    
#     # set simulation configuration
#     # 'T' for max time horizon or 'H' for maximum number of hops
#     term_cond = 10
#     sim_type = 'T' 
#     visualize = True # turn on/off visualization

#     # run a simulation
#     print("Initial Conditions: ", x0)
#     print("Goal: ", xg)
#     xf, objVal = runSimulation(x0 ,xg , term_cond, sim_type, visualize)
#     print("Final Conditions: ", xf)
#     print("Objective Value: ", objVal)
#     print("Euclidean Dist:", np.linalg.norm(x0 -xf))

##############################################################################################

# validation simulation
# waypoints
x0 = [-1 , -1, 0.5, 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.] # initial condition uses rpy
x1 = [0 , 0, 0.5, 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
x2 = [0 , 1, 0.5, 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
x3 = [1 , 0, 0.5, 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
x4 = [1 , 1, 0.5, 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
waypts = [x0, x1, x2, x3, x4]

# parameterization
p = 3 # number hops or time between changing goal points
params = []
for i in range(len(waypts)):
    params.append(p*i)
print(params)

time.sleep(1)

xf, objVal = validationSimulation(x0, waypts, params, 'T', True)
print(xf)

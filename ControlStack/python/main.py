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

# for random support functions
from utils import *

##############################################################################################
# state information
# [11] hopper.q = x, y, z, qw, qx, qy, qz, L, fw1_pos, fw2_pos, fw3_pos
# [10] hopper.v = xdot, ydot, zdot, omega_x, omega_y, omega_z, Ldot, fw1_vel, fw2_vel, fw3_vel 
# [21] x = [hopper.q; hopper.v]

# Sample Space [x,y,z,r,p,y, x_dot, y_dot, z_dot, omega_x, omega_y, omega_z]
# x_s = [x, y, 0.5, 0, 0, 0, xdot, ydot, 0, 0, 0] <--- sample like this
##############################################################################################

def runSimulation(x0, xg, T):
    """ Run a forward simulation for the hopper
    Parameters:
        x0 (list): initial state as 12 dimensional list
        xg (list): goal state as 12 dimensional list
        T (float): time horizon for simulation
    Returns: 
        xf (list): final state as 12 dimensional list
        objVal (float): objective value
    """

    # create controller and sim objects
    c = hopper.Controller()
    s = hopper.Simulator()

    # instantiate thread for controller and sim
    control_thread = threading.Thread(target=call_run, args=(c,))
    sim_thread = threading.Thread(target=call_run_sim, args=(s,))

    # set inital state and goal state, ( pos[3], rpy[3], v[3], omega[3] )
    c.setInitialState(x0)
    c.setGoalState(xg)

    # set simulation stop condition
    sim_duration = T  # number of seconds before sim stops

    # start control and sim threads
    control_thread.start()
    sim_thread.start()

    # run until termination condition
    while c.t_last < sim_duration:
        pass
    c.stopSimulation()

    xf = c.x
    objVal = c.objVal

    # safely shutdown the threads
    c.killSimulation()
    control_thread.join()
    sim_thread.join()

    return xf, objVal

def validationSimulation(x0, waypts, times):
    """ Run a simulation to validate sequecne of waypoints
    Parameters:
        x0 (list): initial state as 12 dimensional list
        xg (list of lists): list of 12 dimensional lists (state waypoints)
        times (list): set of intervals between waypoints
    Returns: 
        xf (list): final state as 12 dimensional list
    """

    # create controller and sim objects
    c = hopper.Controller()
    s = hopper.Simulator()

    # instantiate thread for controller and sim
    control_thread = threading.Thread(target=call_run, args=(c,))
    sim_thread = threading.Thread(target=call_run_sim, args=(s,))

    # set inital state and goal state, ( pos[3], rpy[3], v[3], omega[3] )
    c.setInitialState(x0)
    c.setStateSequence(waypts,times)

    # set simulation stop condition
    sim_duration = times[-1]  # number of seconds before sim stops

    # start control and sim threads
    control_thread.start()
    sim_thread.start()

    # run until termination condition
    while c.t_last < sim_duration:
        pass
    c.stopSimulation()

    xf = c.x
    objVal = c.objVal

    # safely shutdown the threads
    c.killSimulation()
    control_thread.join()
    sim_thread.join()

    return xf

##############################################################################################

# test drive the simulation
# for i in range(5):

#     print("*" * 50)
#     print("Test Drive: ", i)

#     # get random init conditions
#     pos =   [2*rand.random()-1, 2*rand.random()-1, 0.5]
#     rpy =   [0.1*rand.random()-0.05, 0.1*rand.random()-0.05, 0.1*rand.random()-0.05]
#     vel =   [0.1*rand.random()-0.05, 0.1*rand.random()-0.05, 0.1*rand.random()-0.05]
#     omega = [0.1*rand.random()-0.05, 0.1*rand.random()-0.05, 0.1*rand.random()-0.05]

#     x0 = [pos[0], pos[1], pos[2], 
#           rpy[0], rpy[1], rpy[2], 
#           vel[0], vel[1], vel[2], 
#           omega[0], omega[1], omega[2]]

#     # set goal state and time horizon
#     xg = [0 , 0, 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.]
#     T = 1.0

#     # run a simulation
#     print("Initial Conditions: ", x0)
#     print("Goal: ", xg)
#     xf, objVal = runSimulation(x0 ,xg , T)
#     time.sleep(0.1)
#     print("Final Conditions: ", xf)
#     print("Objective Value: ", objVal)

##############################################################################################

# validation simulation
# intial condition
x0 = [-1 , -1, 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.] # iinitial condition uses rpy

x1 = [0 ,   0, 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.] # these use lie algebra elements
x2 = [0 ,   1, 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.]
x3 = [1 ,   0, 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.]
x4 = [1 ,   1, 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.]
waypts = [x1, x2, x3, x4]

T = 3.0
times = []
for i in range(0,len(waypts)):
    times.append(T*i)

xf = validationSimulation(x0, waypts, times)
print(xf)
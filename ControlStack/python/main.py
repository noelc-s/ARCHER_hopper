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

def validationSimulation(x0, waypts, parameterization):
    """ Run a simulation to validate sequence of waypoints
    Parameters:
        x0 (list): initial state as 21 dimensional list
        xg (list of lists): list of 21 dimensional lists (state waypoints)
        times (list): set of intervals between waypoints
    Returns: 
        xf (list): final state as 21 dimensional list
    """

    # create controller and sim objects
    c = hopper.Controller()
    s = hopper.Simulator()

    # instantiate thread for controller and sim
    control_thread = threading.Thread(target=call_run, args=(c,))
    sim_thread = threading.Thread(target=call_run_sim, args=(s,))

    # set inital state and goal state, ( pos[3], rpy[3], v[3], omega[3] )
    c.setInitialState(x0)
    c.setStateSequence(waypts,parameterization)

    # set simulation stop condition
    sim_duration = parameterization[-1]  # number of seconds before sim stops

    # start control and sim threads
    control_thread.start()
    sim_thread.start()

    # run until termination condition
    while c.t_last < sim_duration:
        pass

     # stop simulation
    c.stopSimulation() 

    # get final state and objective value
    xf = c.x
    objVal = c.objVal
    # safely shutdown the threads
    control_thread.join()
    sim_thread.join()

    return xf

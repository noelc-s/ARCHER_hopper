import sys
from pathlib import Path
# assume build directory is in the ControlStack directory
sys.path.insert(0, str(Path(__file__).parent.parent / "build"))
from hopper import *
import hopper
import time
import threading
import numpy as np
import networkx as nx
import multiprocessing
import ctypes
import random as rand
from logging import getLogger
from utils import toEulerAngles, toQuaternion


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
    return xf, c.objVal

# Test this stuff out
x0_ = [0., 0., 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.]
xg_ = [1, 1, 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.]
T_ = 2.0
xf, objVal = runSimulation(x0_,xg_,T_)
print(xf)
print(objVal)
logger = getLogger(__file__)
logger.setLevel("DEBUG")
logger.info(f"Final: {x0_}")
# print metrics
logger.info(f"ObjVal: {objVal}")

# truncate xf down to lower dimension (need to convert quaternion)
# quat  = xf[3:6+1]
# rm_idx = [7,8,9,10,17,18,19,20] # remove L,fw_pos, Ldot, fw_vel
# xf_low_dim = np.delete(xf,rm_idx)
# print("xf low dim: "); print(xf_low_dim)



#######################################################################################

# state dimensionality: 21
# input dimensionality: 4

# sample random states
# Segio
# states t, x, y, z, qw, qx, qy wz, xdot, ydot, zdot, wx, wy, wz, foot_contact(bool), foot_pos, foot_vel, fw_vel_1, fw_vel_2, fw_vel_3
# Noel
# states x, y, z, qw, qx, qy wz, xdot, ydot, zdot, wx, wy, wz, foot_pos, foot_vel, fw_pos_1, fw_pos_2, fw_pos_3, fw_vel_1, fw_vel_2, fw_vel_3
#Should sample
# x,y, 0.5, 1, 0, 0, 0, x_dot, y_dot, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0
num_samples = 64
goal_position = np.array([0.,2.,0.5,0.,0.,0.,0.,0.,0.,0.,0.,0., 0., 0., 0., 0., 0., 0., 0, 0., 0.])
start_position = np.array([0.,0.,0.5,0.,0.,0.,0.,0.,0.,0.,0.,0., 0., 0., 0., 0., 0., 0., 0, 0., 0.])
sample_state_mask = np.array([1, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=bool)
state_template = np.array([0.,0.,0.5,1.,0.,0.,0.,0.,0.,0.,0.,0., 0., 0., 0., 0., 0., 0., 0, 0., 0.])
sample_states_lower = np.array([-5, -5, -1, -1])
sample_states_upper = -sample_states_lower

goal_bias = 0.1
goal_graph_bias = 0.1
goal_random = 1 - goal_bias - goal_graph_bias
init_graph_bias = 0.5

def make_full_state(reduced_state):
    full_state = state_template.copy()
    full_state[sample_state_mask] = reduced_state
    return full_state
G = nx.Graph()
G.add_node(0, state=start_position)
G.add_node(-1, state=goal_position)


def pretend_dynamics(x_init, x_goal):
    """
    x_init: initial state
    x_goal: goal state

    returns: cost, reached_idx
    cost: cost of reaching the goal state
    reached_idx: index of the reached state in the graph
    """
    reached = x_goal #TODO: Sergio get real simulator dynamics
    reached_idx = len(G.nodes)
    G.add_node(len(G.nodes), state=reached)
    cost = np.linalg.norm(x_init - reached) #TODO: Sergio get real MPC cost
    return reached_idx, cost


def simulator_dynamics(x_init, x_goal, tmax=3.0):
    xf, cost = runSimulation(x_init, x_goal, tmax)
    reached_idx = len(G.nodes)
    G.add_node(reached_idx, state=xf)
    return reached_idx, cost


def sample_state():
    init_sample_choice = np.random.choice([0, 1], p=[1-init_graph_bias, init_graph_bias])
    x_init = None
    x_goal = None
    x_init_idx = None
    if init_sample_choice == 0:
        #sample randomly
        x_init = np.random.uniform(low=sample_states_lower, high=sample_states_upper)
        x_init = make_full_state(x_init)
        x_init_idx = len(G.nodes)
        G.add_node(len(G.nodes), state=x_init)
        logger.debug("Sampled random initial state.")
    elif init_sample_choice == 1:
        #sample from graph
        x_init_idx = np.random.choice(G.nodes)
        x_init = G.nodes[x_init_idx]["state"]
        logger.debug("Sampled initial state from graph.")
    else: 
        raise ValueError("init_sample_choice must be 0 or 1")
    goal_sample_choice = np.random.choice([0, 1, 2], p=[goal_random, goal_bias, goal_graph_bias])
    if goal_sample_choice == 0:
        #randomly sample
        x_goal = np.random.uniform(low=sample_states_lower, high=sample_states_upper)
        x_goal = make_full_state(x_goal)
        logger.debug("Sampled random goal state.")
    elif goal_sample_choice == 1:
        # pick goal
        x_goal = goal_position
        logger.debug("Sampled goal state from goal position.")
    elif goal_sample_choice == 2:
        # sample from graph
        x_goal_idx = np.random.choice(G.nodes)
        x_goal = G.nodes[x_goal_idx]["state"]
        logger.debug("Sampled goal state from graph.")
    else:
        raise ValueError("goal_sample_choice must be 0, 1, or 2")
    # if len(G.nodes) < 10:
    #     return 
    # else:
    return np.stack([x_init, x_goal], axis=0), x_init_idx


for i in range(num_samples):
    # sample initial state and goal
    sample, x_init_idx = sample_state()
    reached_idx, cost = runSimulation(sample[0], sample[1], 3.0)
    G.add_edge(x_init_idx, reached_idx, cost=cost, goal=sample[1])
    logger.info(f"Evaluating Sample {i}: {sample}")

# time.sleep(1)

# print(c.programState_)
# print(c.TX_torques)
# time.sleep(1)
# c.startSimulation()
# time.sleep(3)
# c.startSimulation()
# print(c.TX_torques)



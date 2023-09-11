#%%
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
import logging
import time
from utils import toEulerAngles, toQuaternion
import matplotlib.pyplot as plt
import os
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


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)
logger.setLevel(logging.DEBUG)
# Test this stuff out
x0_ = [0., 0., 0.5,
       0., 0., 0., 1.,
       0., 0., 0., 0., 
       0., 0., 0., 0., 
       0., 0., 0,
       0., 0., 0.]
xg_ = [1, 1, 0.5, 
       0., 0., 0., 1.,
       0., 0., 0., 0., 
       0., 0., 0., 0., 
       0., 0., 0,
       0., 0., 0.]
T_ = 3
# logger.debug("Running Test Simulationl.")
# xf, objVal = runSimulation(x0_, xg_, T_, 'H',True)


# logger.info(f"Final: {xf}")
# # print metrics
# logger.info(f"ObjVal: {objVal}")
# logger.info("F")

# validation simulation
# waypoints
x0 = [-1 , -1, 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.] # initial condition uses rpy
x1 = [0 , 0, 0.5, 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
x2 = [0 , 1, 0.5, 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
x3 = [1 , 0, 0.5, 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
x4 = [1 , 1, 0.5, 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
waypts = [x0, x1, x2, x3, x4]

# parameterization
p = 3 # number hops or time between changing goal points
params = []
for i in range(len(waypts)):
    params.append(p*i)
print(params)

xf, objVal = validationSimulation(x0, waypts, params, 'H', False)
logger.info(f"Final: {xf}")
# print metrics
logger.info(f"ObjVal: {objVal}")
logger.info("F")

#######################################################################################

# state dimensionality: 21
# input dimensionality: 4

#  sample random states
# state = [x, y, z, qx, qy, qz, qw, L, fw1, fw2, fw3,  xdot, ydot, zdot, wx, wy, wz, Ldot, fw1dot, fw2dot, fw3dot]
# Should sample
#  x,y, 0.5, 0, 0, 0, 1, 0, 0, 0, 0, x_dot, y_dot, 0, 0, 0, 0, 0, 0, 0 ,0, 0
num_samples = 256
goal_position = np.array([0., 2., 0.5,
                         0., 0., 0., 1.,
                         0.5, 0., 0., 0.,
                         0., 0., 0., 0., 
                         0., 0., 0., 0, 
                         0., 0.])
start_position = np.array([0., 0., 0.5,
                           0.5, 0., 0., 1.,
                           0., 0., 0., 0., 
                           0., 0., 0., 0., 
                           0., 0., 0., 0,
                           0., 0.])
sample_state_mask = np.array([1, 1, 0, 
                              0, 0, 0, 0, 
                              0, 0, 0, 0, 
                              1, 1, 0, 0,                               
                              0, 0, 0, 0, 
                              0, 0], dtype=bool)
state_template = np.array([0., 0., 0.5,
                           0., 0., 0., 1.,
                           0.5, 0., 0., 0.,
                           0., 0., 0., 0.,
                           0., 0., 0., 0,
                           0., 0.])
sample_states_lower_start = np.array([-5., -5., -1., -1.])
sample_states_upper_start = -sample_states_lower_start
sample_states_lower_goal = np.array([-1., -1., -1., -1.])
sample_states_upper_goal = -sample_states_lower_goal

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

def state_metric(x1, x2):
    return np.linalg.norm(x1[[0, 1, 11, 12]] - x2[[0, 1, 11, 12]])

def pretend_dynamics(x_init, x_goal):
    """
    x_init: initial state
    x_goal: goal state

    returns: cost, reached_idx
    cost: cost of reaching the goal state
    reached_idx: index of the reached state in the graph
    """
    reached = runSimulation(x_init, x_goal, 1, 'H', False) #TODO: Sergio get real simulator dynamics
    reached_idx = len(G.nodes)
    G.add_node(len(G.nodes), state=reached)
    cost = np.linalg.norm(x_init - reached) #TODO: Sergio get real MPC cost
    return reached_idx, cost


def simulator_dynamics(x_init, x_goal):
    xf, cost = runSimulation(x_init, x_goal, 1, 'H', False)
    min_dist = np.inf
    reached_idx = None
    for node in G.nodes:
        dist = state_metric(G.nodes[node]["state"], xf)
        if dist < min_dist:
            min_dist = dist
            reached_idx = node
    if min_dist > 1e-3:
        # just add the resulting state to the graph
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
        x_init = np.random.uniform(low=sample_states_lower_start, 
                                   high=sample_states_upper_start)
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
        # randomly sample
        x_goal_offset = np.random.uniform(low=sample_states_lower_goal, 
                                          high=sample_states_upper_goal)
        x_goal_offset = make_full_state(x_goal_offset)
        x_goal = x_init + x_goal_offset
        logger.debug("Sampled random goal state.")
    elif goal_sample_choice == 1:
        # pick goal
        if state_metric(goal_position, x_init) < 1.0:
            x_goal = goal_position
            logger.debug("Sampled goal state from goal position.")
        else:
            x_goal = x_init + (goal_position - x_init) / state_metric(goal_position, x_init)
            logger.debug("Sampled projected goal state.")
    elif goal_sample_choice == 2:
        # sample from graph
        x_goal_idx = np.random.choice(G.nodes)
        x_goal = G.nodes[x_goal_idx]["state"]
        if state_metric(x_goal, x_init) > 1.0:
            x_goal = x_init + (x_goal - x_init) / state_metric(x_goal, x_init)
            logger.debug("Sampled projected goal state from graph.")
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
    logger.debug(f"Sampled Initial Condition: {sample[0]}")
    logger.debug(f"Sampled Goal Condition: {sample[1]}")
    start_time = time.perf_counter()
    reached_idx, cost = simulator_dynamics(sample[0], sample[1])
    end_time = time.perf_counter()
    G.add_edge(x_init_idx, reached_idx, cost=cost, goal=sample[1])
    logger.info(f"Simulator Call took {end_time-start_time} seconds.")
    logger.info(f"Evaluating Sample {i}: {sample}")

#attemot to connect graph by going from every unconnected edje to every other unconnected edge.
original_nodes = list(G.nodes)
for u in original_nodes:

    for v in original_nodes:
        if u == v: continue
        if not G.has_edge(u, v):
            if state_metric(G.nodes[u]["state"], G.nodes[v]["state"]) < 1.0:
                reached_idx, cost = simulator_dynamics(G.nodes()[u]["state"],
                                                       G.nodes[v]["state"])
                G.add_edge(u, v, cost=cost, goal=G.nodes[v]["state"])
                logger.info(f"Attempted to add near edge between {u} and {v} with actual {reached_idx}.")

plt.figure() 
node_pos = np.zeros((len(G.nodes), 2))
node = G.nodes[0]
# for i, node in enumerate(G.nodes):
#     node_pos[i, 0] = G.nodes[node]["state"][0]
#     node_pos[i, 1] = G.nodes[node]["state"][1]

for x0_node, xf_node, data, in G.edges(data=True):
    x0 = G.nodes[x0_node]["state"]
    xf = G.nodes[xf_node]["state"]    
    plt.plot([x0[0], xf[0]], [x0[1], xf[1]], marker='o')
# plt.scatter(node_pos[:, 0], node_pos[:, 1])
os.makedirs('figs', exist_ok=True)
plt.savefig('figs/test.png')

logger.info("Finished Sampling.")


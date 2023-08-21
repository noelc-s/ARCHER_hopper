import sys
sys.path.insert(0, '../build') #this is cursed

import hopper
import time
import threading
import numpy as np
import networkx as networkx

c = hopper.Controller()
x1 = threading.Thread(target=c.run)

sim_duration = 5
# c.resetSimulation([0.,0.,10.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
# above should set the initial state of mujoco, but it's broken
# (there is some janky programState_ state machine logic)
# solution: add a send (over socket) command to the start of the controller,
# before the for loop, and a read to the start of the simulator.cpp, before the simulation
# step 1: make the controller read the initial condition from yaml
# step 2: get controller to send IC to simulator (over socket), and have the simulator run as normal
# step 3: use python to set the state of controller (instead of reading from yaml)

# print(c.TX_torques)
x1.start()
while c.t_last < sim_duration:
    pass
c.stopSimulation()
print(c.state)
# state dimensionality: 21
# input dimensionality: 4

# sample random states
state_lower = -np.ones(shape=(1,21))
state_upper = np.ones(shape=(1,21))
sample = np.random.sample(size=state_lower.shape)
sample_states = (state_upper - state_lower)*sample + state_lower

for i in range(sample_states.shape[0]):
    print(f"[INFO] evaluating state {sample_states[i]}")
    # 
#time.sleep(1)

#print(c.programState_)
# print(c.TX_torques)
#time.sleep(1)
#c.startSimulation()
#time.sleep(3)
# c.startSimulation()
# print(c.TX_torques)


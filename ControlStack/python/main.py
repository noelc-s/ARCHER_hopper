import sys
sys.path.insert(0, '../build') #this is cursed

from hopper import *
import hopper
import time
import threading
import numpy as np
import networkx as networkx

########################################################################
# step 1: turn the Simulator.cpp into a class (the same way Controller was turned into a class). This means that the "main" script should just run simulator.run()
# step 2: make a simulator python class that can call the run
# step 3: instantiate that here, and call the run function
# pseudo-code (python): 
#   s = hopper.Simulator()
#   x2 = threading.Thread(target=s.run)
#   c.setInitialState(...)
#   x1.start()
#   x2.start()
########################################################################

# create controller and sim objects
c = hopper.Controller()
s = hopper.Simulator()

# instantiate thread for controller and sim
control_thread = threading.Thread(target=call_run, args=(c,))
sim_thread = threading.Thread(target=call_run_sim, args=(s,))

# set simulation duration
sim_duration = 1

# set inital state
c.setInitialState([0.,0.,0.5,0.,0.,0.,0.,0.,0.,0.,0.,0.])

# start control and sim threads
control_thread.start()
sim_thread.start()
# call_sim_run(s)


# simulate until sim_duration 
while c.t_last < sim_duration:
    pass
c.stopSimulation()
print(c.state)
#try:
#    c.killSimulation()
#except:
#    pass

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


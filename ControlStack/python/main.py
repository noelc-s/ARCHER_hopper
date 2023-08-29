import sys
sys.path.insert(0, '../build') #this is cursed

from hopper import *
import hopper
import time
import threading
import numpy as np
import networkx as networkx

################################################################################################################
# [11] hopper.q = x, y, z, qw, qx, qy, qz, L, fw1_pos, fw2_pos, fw3_pos
# [10] hopper.v = xdot, ydot, zdot, omega_x, omega_y, omega_z, Ldot, fw1_vel, fw2_vel, fw3_vel 
# [21] x = hopper.q, hopper.v
# [21] state = t,x,y,z,qw,qx,qy,qz,xdot,ydot,zdot,omega_x,omega_y,omega_z,contact,L,Ldot,fw1_vel,fw2_vel,fw3_vel  
################################################################################################################

# create controller and sim objects
c = hopper.Controller()
s = hopper.Simulator()

# instantiate thread for controller and sim
control_thread = threading.Thread(target=call_run, args=(c,))
sim_thread = threading.Thread(target=call_run_sim, args=(s,))

# set simulation duration
sim_duration = 2

# set inital state, ( pos[3], rpy[3], v[3], omega[3] ) 
c.setInitialState([0., 0., 0.5, 1., 0.5, -0.2, 0., 0., 0., 0., 0., 0.])

# start control and sim threads
control_thread.start()
sim_thread.start()
# call_sim_run(s)


print("Intial: "); print(c.x)

# simulate until sim_duration 
while c.t_last < sim_duration:
    pass
c.stopSimulation()

print("Final: "); print(c.x)

# state dimensionality: 21
# input dimensionality: 4

# sample random states
#state_lower = -np.ones(shape=(1,21))
#state_upper = np.ones(shape=(1,21))
#sample = np.random.sample(size=state_lower.shape)
#sample_states = (state_upper - state_lower)*sample + state_lower
#
#for i in range(sample_states.shape[0]):
#    print(f"[INFO] evaluating state {sample_states[i]}")
    # 
#time.sleep(1)

#print(c.programState_)
# print(c.TX_torques)
#time.sleep(1)
#c.startSimulation()
#time.sleep(3)
# c.startSimulation()
# print(c.TX_torques)


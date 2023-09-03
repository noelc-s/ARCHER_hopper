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

# for random support functions
from utils import *

c = hopper.Controller()
s = hopper.Simulator()

class thread_with_exception(threading.Thread):
    def __init__(self, name):
        threading.Thread.__init__(self)
        self.name = name
             
    def run(self):
 
        # target function of the thread class
        try:
            c.run()
        finally:
            print('ended')
          
    def get_id(self):
 
        # returns id of the respective thread
        if hasattr(self, '_thread_id'):
            return self._thread_id
        for id, thread in threading._active.items():
            if thread is self:
                return id
  
    def raise_exception(self):
        thread_id = self.get_id()
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id,
              ctypes.py_object(SystemExit))
        if res > 1:
            ctypes.pythonapi.PyThreadState_SetAsyncExc(thread_id, 0)
            print('Exception raise failure')

##############################################################################################
# state information
# [11] hopper.q = x, y, z, qw, qx, qy, qz, L, fw1_pos, fw2_pos, fw3_pos
# [10] hopper.v = xdot, ydot, zdot, omega_x, omega_y, omega_z, Ldot, fw1_vel, fw2_vel, fw3_vel 
# [21] x = [hopper.q; hopper.v]

# Sample Space [x,y,z,r,p,y, x_dot, y_dot, z_dot, omega_x, omega_y, omega_z]
# x_s = [x, y, 0.5, 0, 0, 0, xdot, ydot, 0, 0, 0] <--- sample like this
##############################################################################################

# create controller and sim objects
# instantiate thread for controller and sim
control_thread = threading.Thread(target=call_run, args=(c,))
#control_thread = thread_with_exception('Thread 1')
sim_thread = threading.Thread(target=call_run_sim, args=(s,))

#control_thread = multiprocessing.Process(target=c.run)
#sim_thread = multiprocessing.Process(target=s.run)

# set inital state and goal state, ( pos[3], rpy[3], v[3], omega[3] )
c.setInitialState([0., 0., 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.])
c.setGoalState([1, 1, 0.5, 0., 0., 0., 0., 0., 0., 0., 0., 0.])

x0 = c.initialCondition_
xg = c.goalState_

# start control and sim threads
control_thread.start()
sim_thread.start()

# set simulation stop condition
sim_duration = 1  # number of seconds before sim stops
max_hops = 3      # number of apex-to-apex hops before sim stops
sim_type = 0      # 0 = sim_suration, 1 = max_hops

# simulate until the stopping condition
print("Initial: "); print(x0)

# by sim duration
if sim_type == 0: 
    while c.t_last < sim_duration:
        print(c.objVal)
    c.stopSimulation()
    xf = c.x

# by max hops
elif sim_type == 1:
    while c.t_last < sim_duration:
        pass
    c.stopSimulation()
    xf = c.x

#sim_thread.terminate()

print("Final: "); print(xf)

# print metrics
print("ObjVal: "); print(c.objVal)

# truncate xf down to lower dimension (need to convert quaternion)
# quat  = xf[3:6+1]
rm_idx = [7,8,9,10,17,18,19,20] # remove L,fw_pos, Ldot, fw_vel
xf_low_dim = np.delete(xf,rm_idx)
print("xf low dim: "); print(xf_low_dim)


#control_thread.terminate()
#control_thread.raise_exception()
c.killSimulation()
s.killSimulation()
#control_thread.join()
#sim_thread.join()

#######################################################################################

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


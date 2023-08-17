import sys
sys.path.insert(0, '../build')

import hopper
import time
import threading
import numpy as np

c = hopper.Controller()
x = threading.Thread(target=c.run)

print(c.t_last)
x.start()
time.sleep(3)
print(c.TX_torques)
c.stopSimulation()
time.sleep(1)
c.resetSimulation([0.,0.,10.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
print(c.TX_torques)
time.sleep(1)
c.startSimulation()
#time.sleep(3)
# c.startSimulation()
# print(c.TX_torques)

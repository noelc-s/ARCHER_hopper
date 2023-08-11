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
print("hello")
c.stopSimulation()
time.sleep(3)
c.resetSimulation([0.,0.,1.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
# c.startSimulation()
# print(c.TX_torques)

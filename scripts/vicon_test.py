import rosbag
import numpy as np
from std_msgs.msg import Int32, String
import matplotlib.pyplot as plt
from vicon_functions import *
import time

bag = rosbag.Bag('./bagfiles/2018-02-23-20-57-31.bag')


start = time.time()
output = parseViconPose(bag)
end = time.time()

print(end-start)

output[5] -= output[5][0]
output[3] -= output[3][0]

plt.figure()
plt.plot(output[5][:,0])
plt.plot(output[5][:,1])
plt.plot(output[5][:,2])
plt.show()

plt.figure()
plt.plot(output[3][:,0])
plt.plot(output[3][:,1])
plt.plot(output[3][:,2])
plt.show()
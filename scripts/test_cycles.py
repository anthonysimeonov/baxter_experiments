import rosbag
import numpy as np
from std_msgs.msg import Int32, String
import matplotlib.pyplot as plt

bag = rosbag.Bag('./bag/cycles_3.bag')

positions = []
states = []
tstate = []
tposition = []
for topic, msg, t in bag.read_messages(topics = ['/board_pose/pose', '/board_pose/cycle_time_2']):
	# print(msg.pose.position)
	if topic == '/board_pose/pose':
		position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
		positions.append(position)
		tposition.append(msg.header.stamp.to_sec())# / 10**6)
	if topic == '/board_pose/cycle_time_2':
		# state = int(msg.data)
		# states.append(state)
		tstate.append(msg.data.to_sec()) #/ 10**6)
			# print(t.nsecs)
			# print(t.secs)

tstate = np.array(tstate)
tposition = np.array(tposition)
states = np.array(states)
positions = np.array(positions)


tposition -= tstate[0]
tstate -= tstate[0]

print(tstate)
print(states)

rep = np.repeat(tposition[:, np.newaxis], tstate.shape[0], axis = 1)
delta = rep - tstate
error = delta * delta
best_ind = np.argmin(error, axis = 0)

# positions1 = positions[best_ind[1]:best_ind[2]]
# positions2 = positions[best_ind[3]:best_ind[4]]
# tpositions1 = tposition[best_ind[1]:best_ind[2]]
# tpositions2 = tposition[best_ind[3]:best_ind[4]]

# positionsMat = np.zeros((len(best_ind)/2 + 1, positions.shape[0], positions.shape[1]))
# for i in range(positionsMat.shape[0]):
# 	positionsMat[i] = positions[best_ind[2*i]:best_ind[2*i+1]]

positions1 = positions[best_ind[0]:best_ind[1]]
positions2 = positions[best_ind[2]:best_ind[3]]
positions3 = positions[best_ind[4]:best_ind[5]]
positions4 = positions[best_ind[6]:best_ind[7]]
positions5 = positions[best_ind[8]:best_ind[9]]

tpositions1 = tposition[best_ind[0]:best_ind[1]]
tpositions1 = tposition[best_ind[2]:best_ind[3]]
tpositions3 = tposition[best_ind[4]:best_ind[5]]
tpositions4 = tposition[best_ind[6]:best_ind[7]]
tpositions5 = tposition[best_ind[8]:best_ind[9]]

for i in range(1,6):
	var = "positions%d" %i
	print(len(eval(var)))

max_ind = min(positions1.shape[0], positions2.shape[0])
positions1 = positions1[:max_ind]
positions2 = positions2[:max_ind]
tpositions1 = tpositions1[:max_ind]
tpositions2 = tpositions2[:max_ind]

norm = np.linalg.norm(positions1-positions2, axis = 1)
np.average()
# print(tposition)

#--------------------------------------------
#find closest index for transitions
#---------------------------------------------


# pos_mean = np.mean(positions, axis = 0)
# pos_std = np.std(positions, axis = 0)
import rosbag
import numpy as np
from std_msgs.msg import Int32, String
import matplotlib.pyplot as plt

bag = rosbag.Bag('./bagfiles/repeat.bag')

positions1 = []
rotations1 = []
positions2 = []
rotations2 = []
tposition1 = []
tposition2 = []

states = []
tstate = []


i = 0
for topic, msg, t in bag.read_messages(topics = ['/tf', '/board_pose/cycle_time_2']):
	print(i)
	i += 1
	if i > 2:
		break
	# print(msg.pose.position)
	if topic == 'tf':
		position = np.array([msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z])
		rotation = np.array([msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].rotation.translation.z])
		msg_time = msg.transforms[0].header.stamp.to_sec()

		frame_id = msg.transforms[0].child_frame_id

		if frame_id == 'j1_dim/base_link':
			positions1.append(position)
			rotations1.append(rotation)
			tposition1.append(msg_time)
		if frame_id == 'j2_dim/base_link':
			positions2.append(position)
			rotations2.append(rotation)
			tposition2.append(msg_time)# / 10**6)

	if topic == '/board_pose/cycle_time_2':
		# state = int(msg.data)
		# states.append(state)
		tstate.append(msg.data.to_sec()) #/ 10**6)
			# print(t.nsecs)
			# print(t.secs)



print(i)

tstate = np.array(tstate)
states = np.array(states)

tposition1 = np.array(tposition1)
positions1 = np.array(positions1)

tposition2 = np.array(tposition2)
positions2 = np.array(positions2)

'''
#tposition -= tstate[0]
#tstate -= tstate[0]

#print(tstate)

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

positions_parsed = []
tpositions_parsed = []
run_num = []

#actual code
for i in range(1,len(best_ind)/2):
	x = positions[best_ind[i*2]:best_ind[i*2+1]]
	if (np.max(x) < 5000):
		positions_parsed.append(positions[best_ind[i*2]:best_ind[i*2+1]])
		tpositions_parsed.append(tposition[best_ind[i*2]:best_ind[i*2+1]])
		run_num.append(i)

# for i in range(1,len(best_ind)/2):
# 	x = positions[best_ind[i*2-1]:best_ind[i*2]]
# 	if (np.max(x) < 5000):
# 		positions_parsed.append(positions[best_ind[i*2-1]:best_ind[i*2]])
# 		tpositions_parsed.append(tposition[best_ind[i*2-1]:best_ind[i*2]])
# 		run_num.append(i)

#quick test

# for i in range(len(best_ind)/2):
# 	x = positions[best_ind[i*2]:best_ind[i*2+1]]
# 	x[x > 5000] = 0
# 	#if (np.max(x) < 5000):
# 	positions_parsed.append(x)
# 	tpositions_parsed.append(tposition[best_ind[i*2]:best_ind[i*2+1]])
# 	run_num.append(i)



# above = []
# below = []
# for i in range(len(run_num)):
# 	for j in range(i,len(run_num)):
# 		base = positions_parsed[i]
# 		current = positions_parsed[j]



min_length = len(min(positions_parsed, key=len))


for i in range(len(positions_parsed)):
	positions_parsed[i] = positions_parsed[i][:min_length]

color_idx = np.linspace(0, 1, len(run_num))

plt.figure(0)

for i,col in zip(range(len(positions_parsed)), color_idx):
	plt.subplot(311)
	plt.plot(positions_parsed[i][:,0], color = plt.cm.rainbow(col), lw = 1, label = str(run_num[i]))

	plt.subplot(312)
	plt.plot(positions_parsed[i][:,1], color = plt.cm.rainbow(col), lw = 1, label = str(run_num[i]))

	plt.subplot(313)
	plt.plot(positions_parsed[i][:,2], color = plt.cm.rainbow(col), lw = 1, label = str(run_num[i]))


plt.legend()
plt.show()

# positions1 = positions[best_ind[0]:best_ind[1]]
# positions2 = positions[best_ind[2]:best_ind[3]]
# positions3 = positions[best_ind[4]:best_ind[5]]
# positions4 = positions[best_ind[6]:best_ind[7]]
# positions5 = positions[best_ind[8]:best_ind[9]]

# tpositions1 = tposition[best_ind[0]:best_ind[1]]
# tpositions2 = tposition[best_ind[2]:best_ind[3]]
# tpositions3 = tposition[best_ind[4]:best_ind[5]]
# tpositions4 = tposition[best_ind[6]:best_ind[7]]
# tpositions5 = tposition[best_ind[8]:best_ind[9]]


# positions1 = positions_parsed[0]
# positions2 = positions_parsed[1]
# tpositions1 = tpositions_parsed[0]
# tpositions2 = tpositions_parsed[1]


# #for i in range(1,6):
# #	var = "positions%d" %i
# #	print(len(eval(var)))


# max_ind = min(positions1.shape[0], positions2.shape[0])
# positions1 = positions1[:max_ind]
# positions2 = positions2[:max_ind]
# tpositions1 = tpositions1[:max_ind]
# tpositions2 = tpositions2[:max_ind]

# norm = np.linalg.norm(positions1-positions2, axis = 1)
# plt.plot(positions1)
# plt.plot(positions2)
# plt.show()

# print(np.array(positions_parsed).shape)
# print(np.array(tpositions_parsed).shape)

# # print(tposition)

# #--------------------------------------------
# #find closest index for transitions
# #---------------------------------------------


# # pos_mean = np.mean(positions, axis = 0)
# # pos_std = np.std(positions, axis = 0)

'''
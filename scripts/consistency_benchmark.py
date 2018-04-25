import rosbag
import numpy as np
from std_msgs.msg import Int32, String

bag = rosbag.Bag('one_joint_one_bag.bag')


# def parseViconPose(rosBag):
positions1 = []
rotations1 = []
positions2 = []
rotations2 = []
tposition1 = []
tposition2 = []
#
states = []
tstate = []
#
#
i = 0
for topic, msg, t in bag.read_messages(topics = ['/tf', '/board_pose/cycle_time']):
	#if i > 10:
	#	break
	#print msg
	#print topic
	# print(msg.pose.position)
	if topic == '/tf':
		position = np.array([msg.transforms[0].transform.translation.x, msg.transforms[0].transform.translation.y, msg.transforms[0].transform.translation.z])
		rotation = np.array([msg.transforms[0].transform.rotation.w, msg.transforms[0].transform.rotation.x, msg.transforms[0].transform.rotation.y, msg.transforms[0].transform.rotation.z])
		msg_time = msg.transforms[0].header.stamp.to_sec()

		frame_id = msg.transforms[0].child_frame_id

		#print frame_id
		if frame_id == 'j1_dim/base_link':
			#print msg
			positions1.append(position)
			rotations1.append(rotation)
			tposition1.append(msg_time)
		if frame_id == 'j2_dim/base_link':
			positions2.append(position)
			rotations2.append(rotation)
			tposition2.append(msg_time)# / 10**6)
			i += 1
			#print(i)

	if topic == '/board_pose/cycle_time':
		# state = int(msg.data)
		# states.append(state)
		tstate.append(msg.data.to_sec()) #/ 10**6)
			# print(t.nsecs)
			# print(t.secs)



tstate = np.array(tstate)
states = np.array(states)

tposition1 = np.array(tposition1)
positions1 = np.array(positions1)
rotations1 = np.array(rotations1)

tposition2 = np.array(tposition2)
positions2 = np.array(positions2)
rotations2 = np.array(rotations2)

	return tstate, states, tposition1, positions1, rotations1, tposition2, positions2, rotations2


def parseViconPoseTF(rosBag):
	positions1 = []
	rotations1 = []
	positions2 = []
	rotations2 = []
	tposition1 = []
	tposition2 = []

	states = []
	tstate = []


	i = 0
	for topic, msg, t in rosBag.read_messages(topics = ['/tf', '/board_pose/cycle_time']):
		# if i > 10:
		# 	break
		# print msg
		# print(msg.pose.position)
		if topic == '/tf':
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
				i += 1
				#print(i)

		if topic == '/board_pose/cycle_time':
			# state = int(msg.data)
			# states.append(state)
			tstate.append(msg.data.to_sec()) #/ 10**6)
				# print(t.nsecs)
				# print(t.secs)



	tstate = np.array(tstate)
	states = np.array(states)

	tposition1 = np.array(tposition1)
	positions1 = np.array(positions1)

	tposition2 = np.array(tposition2)
	positions2 = np.array(positions2)
	return tstate, states, tposition1, positions1, tposition2, positions2

def main():
	#want to parse bag files and save data in .csv that can be easier loaded later
	root_dir = ''
	filename = root_dir + 'one_joint_one_bag.bag'
	bag = rosbag.Bag(filename)

	global tstate
	global temp, tposition1, positions1, tposition1, positions2
	tstate, temp, tposition1, positions1, tposition1, positions2 = parseViconPoseTF(bag)


if __name__ == '__main__':
	main()

#!/usr/bin/env python
"""
 Inherited from Rethink Robotics' Trajectory class

 Changes:

"""

from __future__ import print_function

import numpy as np

import argparse
import operator
import sys
import threading
import time

from bisect import bisect
from copy import copy
from os import path

import rospy

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import std_msgs.msg as std_msgs

from baxter_experiments.joint_trajectory import Trajectory
import baxter_examples
import baxter_interface
import rospy

from baxter_interface import CHECK_VERSION

import geometry_msgs.msg as geometry_msgs
import gazebo_msgs.msg as gazebo_msgs
import sensor_msgs.msg as sensor_msgs
import baxter_core_msgs.msg as baxter_core_msgs

import pickle

class RolloutExecuter(Trajectory):
    '''
    Class inherited from Trajectory, modified to support the following capabilities:
    - load the augmented data from a record file (Vicon positions)
    - rolling out recorded trajectories as a discrete chain of start -> goal movements
    - subscribe to Vicon motion capture topic and compute rewards based on world frame error
    - interfaces with an RL agent class, sending and receiving data
    - computes an additive "action" for the robot to take and sends that to the controllers
    '''

    def __init__(self, debug=False, goal_type='API', state_type='OL'):

        self.debug = debug
        self.goal_type = goal_type  #default API
        self.state_type = state_type  #default OL = open loop

        if self.goal_type == 'trajectory':
            super(RolloutExecuter, self).__init__()
        #create our action server clients
        self._left_client = actionlib.SimpleActionClient(
            'robot/limb/left/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._right_client = actionlib.SimpleActionClient(
            'robot/limb/right/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )

        #param namespace
        self._param_ns = '/rsdk_joint_trajectory_action_server/'

        #create our goal request
        self._l_goal = FollowJointTrajectoryGoal()
        self._r_goal = FollowJointTrajectoryGoal()

        #gripper goal trajectories
        self._l_grip = FollowJointTrajectoryGoal()
        self._r_grip = FollowJointTrajectoryGoal()

        #limb interface - current angles needed for start move
        self._l_arm = baxter_interface.Limb('left')
        self._r_arm = baxter_interface.Limb('right')

        #flag to signify the arm trajectories have begun executing
        self._arm_trajectory_started = False
        #reentrant lock to prevent same-thread lockout
        self._lock = threading.RLock()

        #gazebo subscription and world frame pose dictionary for rewards
        self.gazebo_sub = rospy.Subscriber('gazebo/link_states', gazebo_msgs.LinkStates, self.gazebo_callback)
        self.end_effector_pos = None
        self.end_effector_desired = dict()

        #robot subscription for state information (dictionary of lists indexed by timestep)
        self.robot_state_sub = rospy.Subscriber('/robot/limb/right/gravity_compensation_torques', baxter_core_msgs.SEAJointState, self.robot_state_callback)
        self.robot_state_current = dict()
        # self.robot_joint_names = None
        self.robot_joint_names = ['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2']
        self.state_keys = ['theta_command', 'velocity', 'torque', 'theta_measure']
        if self.state_type == 'CL':
            self.state_keys.append('world_pose')

        #world frame error in current end effector position and desired from parsed file
        self.world_error = []

        #publish timing topics
        self.timing_pub_state = rospy.Publisher('/cycle_bool', std_msgs.Bool, queue_size = 1)
        self.timing_msg_state = std_msgs.Bool()
    	self.timing_pub_time = rospy.Publisher('/cycle_time', std_msgs.Time, queue_size = 1)
    	self.timing_msg_time = std_msgs.Time()

        #maintain dictionaries of variables to dump at the end or query
        self.gazebo_dump = []
        self.state_dump = dict()
        self.action_dump = []
        self.reward_dump = []

        self.programmed_positions = dict()
        self.iterations = 1000
        self.current = 0

    def init_state_vector(self):
        """
        method to initialize the dictionary of robot states

        self.state_keys is initialized in __init__, includes 'theta/theta_0/torque'
        self.robot_joint_names are the Baxter joint names
        the state vector is a dictionar in form state = {key1: {name1: [...], name2: [...]}, key2: {}}
        """
        for key in self.state_keys:
            self.robot_state_current[key] = dict()
            self.state_dump[key] = dict()

            for name in self.robot_joint_names:
                self.robot_state_current[key][name] = 0
                self.state_dump[key][name] = []

        if self.debug:
            print("State dictionary keys: \n")
            print(self.robot_state_current.keys())
            print("\n")


    def gazebo_callback(self, data):
        """
        Callback function for gazebo subscriber, fills attribute of current gazebo position
        of end effector
        """
        self.end_effector_pos = [data.pose[-1].position.x, data.pose[-1].position.y, data.pose[-1].position.z]

    def robot_state_callback(self, data):
        """
        Put data from topic msg in current state attribute

        Need to do some fun indexing to deal with the joint names and order that
        the values come in from the msg

        TODO: Decide how to implement the delta_theta as part of the state
        """
        for name in self.robot_joint_names:
            try:
                idx = data.name.index(name)
                self.robot_state_current['torque'][name] = data.actual_effort[idx]
                # self.robot_state_current['theta_command'][name] = data.commanded_position[idx]
                self.robot_state_current['velocity'][name] = data.actual_velocity[idx]
                self.robot_state_current['theta_measure'][name] = data.actual_position[idx]
                self.robot_state_current['theta_command'][name] = self.local_goal_r[name]

            except ValueError:
                pass

            #TODO put the world frame pose in when using closed loop mode

    def publish_bool(self, trajectory_state):
        if trajectory_state:
            self.timing_msg_state.data = 1
        else:
            self.timing_msg_state.data = 0
        self.timing_msg_time = rospy.Time.now()
        self.timing_pub_state.publish(self.timing_msg_state)
        self.timing_pub_time.publish(self.timing_msg_time)

    def append_dump(self):
        """
        Fill out the internal variables that are dumped with pickle at the end
        """
        for key in self.state_dump.keys():
            for name in self.state_dump[key].keys():
                self.state_dump[key][name].append(self.robot_state_current[key][name])

        self.gazebo_dump.append(self.end_effector_pos)
        #fill action and reward

    def _clean_line(self, line, joint_names, link_names):
        """
        Cleans a single line of recorded joint positions and world frame pose

        @param line: the line described in a list to process
        @param joint_names: joint name keys
        @param link_names: link name keys

        @return command_joints: returns dictionary {joint: value} of valid commands
        @return world_links: return dictionary {link: value} of end_eff poses
        @return line_joints: returns list of current line values (joint values) stripped of commas
        @return line_links: returns list of current line values (link values)
        """
        def try_float(x):
            try:
                return float(x)
            except ValueError:
                return None
        #convert the line of strings to a float or None
        line = [try_float(x) for x in line.rstrip().split(',')]
        line_joints = line[:len(joint_names)]
        line_links = line[len(joint_names):]

        #zip the values with the joint names
        combined_joints = zip(joint_names[1:], line_joints[1:])
        combined_links = zip(link_names, line_links)

        #take out any tuples that have a none value
        cleaned_joints = [x for x in combined_joints if x[1] is not None]
        cleaned_links = [x for x in combined_links if x[1] is not None]

        #convert it to a dictionary with only valid commands
        command_joints = dict(cleaned_joints)
        world_links = dict(cleaned_links)
        return (command_joints, world_links, line_joints, line_links)


    def parse_file(self, filename):
        """
        Parses input file into FollowJointTrajectoryGoal format
        and adds the vicon states to a list for reward computation

        @param filename: input filename
        """
        if self.debug:
            print("STARTING FILE PARSE\n")
        #open recorded file
        with open(filename, 'r') as f:
            lines = f.readlines()
        #read joint names specified in file
        joint_names = lines[0].rstrip().split(',')
        link_names = []
        #parse joint names for the left and right limbs
        for name in joint_names:
            if 'left' == name[:-3]:
                self._l_goal.trajectory.joint_names.append(name)
            elif 'right' == name[:-3]:
                self._r_goal.trajectory.joint_names.append(name)
            elif 'gazebo' == name[:-3]:
                self.end_effector_desired[name] = []
                # joint_names.remove(name)  #separate joint names from link names
                link_names.append(name)
        for link_ind in range(len(link_names)):
            joint_names.pop(-1)
        if self.debug:
            print("joint names:\n")
            print(joint_names)
            print("\n")
            print("link_names:\n")
            print(link_names)
            print("\n")

        #set self.joint_names attribute for filling state vector
        # self.robot_joint_names = joint_names

        def find_start_offset(pos):
            #create empty lists
            cur = []
            cmd = []
            dflt_vel = []
            vel_param = self._param_ns + "%s_default_velocity"
            #for all joints find our current and first commanded position
            #reading default velocities from the parameter server if specified
            for name in joint_names:
                if 'left' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._l_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
                elif 'right' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._r_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)


            diffs = map(operator.sub, cmd, cur)
            diffs = map(operator.abs, diffs)
            #determine the largest time offset necessary across all joints
            offset = max(map(operator.div, diffs, dflt_vel))
            return offset

        for idx, values in enumerate(lines[1:]):
            #clean each line of file
            cmd, links, values_joints, values_links = self._clean_line(values, joint_names, link_names)
            #find allowable time offset for move to start position
            if idx == 0:
                # Set the initial position to be the current pose.
                # This ensures we move slowly to the starting point of the
                # trajectory from the current pose - The user may have moved
                # arm since recording
                cur_cmd = [self._l_arm.joint_angle(jnt) for jnt in self._l_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'left', 0.0)
                cur_cmd = [self._r_arm.joint_angle(jnt) for jnt in self._r_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'right', 0.0)
                start_offset = find_start_offset(cmd)
                # Gripper playback won't start until the starting movement's
                # duration has passed, and the actual trajectory playback begins
                self._slow_move_offset = start_offset
                self._trajectory_start_offset = rospy.Duration(start_offset + values_joints[0])
            #add a point for this set of commands with recorded time
            cur_cmd = [cmd[jnt] for jnt in self._l_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'left', values_joints[0] + start_offset)
            cur_cmd = [cmd[jnt] for jnt in self._r_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'right', values_joints[0] + start_offset)
            cur_cmd = [cmd['left_gripper']]
            self._add_point(cur_cmd, 'left_gripper', values_joints[0] + start_offset)
            cur_cmd = [cmd['right_gripper']]
            self._add_point(cur_cmd, 'right_gripper', values_joints[0] + start_offset)

            #add the world frame poses
            for key in links.keys():
                self.end_effector_desired[key].append(links[key])

        if self.debug:
            print("---------------------\n")
            print("Desired world frame dict, parsed from file:\n")
            print(self.end_effector_desired.keys())
            print("---------------------\n")

    def make_local_goal(self, k):
        """
        Makes a local trajectory action requst goal based on
        the k and k+1 indices in the overall goal list

        @parak k: index of start point in the overall list of points
        """
        #make a dictionary for joint angles for API

        if self.goal_type == 'API':
            self.local_goal_r = dict(zip(
                    self._r_goal.trajectory.joint_names,
                    [0, (np.sin(self.current * 2 * np.pi/500) * np.pi/4), 0, 0, 0, 0, 0]))

            self.local_goal_l = dict(zip(
                    self._l_goal.trajectory.joint_names,
                    [0, 0, 0, 0, 0, 0, 0]))

        #make a goal request for action server
        elif self.goal_type == 'trajectory':
            self.local_goal_r = FollowJointTrajectoryGoal()
            self.local_goal_l = FollowJointTrajectoryGoal()

            self.local_goal_r.trajectory.joint_names = self._r_goal.trajectory.joint_names
            self.local_goal_l.trajectory.joint_names = self._l_goal.trajectory.joint_names

            for i in range(2):
                self.local_goal_r.trajectory.points.append(self._r_goal.trajectory.points[k+i])
                self.local_goal_l.trajectory.points.append(self._l_goal.trajectory.points[k+i])
        print("goal made\n")

    def wait(self):
        """
        Waits for and verifies trajectory execution result
        """
        #create a timeout for our trajectory execution
        #total time trajectory expected for trajectory execution plus a buffer

        #only change is the local goal
        last_time = self.local_goal_r.trajectory.points[-1].time_from_start.to_sec()
        time_buffer = rospy.get_param(self._param_ns + 'goal_time', 0.0) + 1.5
        timeout = rospy.Duration(self._slow_move_offset +
                                 last_time +
                                 time_buffer)

        l_finish = self._left_client.wait_for_result(timeout)
        r_finish = self._right_client.wait_for_result(timeout)
        l_result = (self._left_client.get_result().error_code == 0)
        r_result = (self._right_client.get_result().error_code == 0)
        #verify result
        if all([l_finish, r_finish, l_result, r_result]):
            return True
        else:
            msg = ("Trajectory action failed or did not finish before "
                   "timeout/interrupt.")
            rospy.logwarn(msg)
            return False

    def send_goal(self):
        """
        Sends FollowJointTrajectoryAction request

        @param r/l_goal_k: input single trajectory goal at k index from _r/l_goal.trajectory.points
        """
        if self.debug:
            print("goals:\n")
            print("\n-----------------------------\n")
            print(self.local_goal_r)
            print("\n")

        if self.goal_type == 'API':
            self._l_arm.set_joint_positions(self.local_goal_l, raw=False)
            self._r_arm.set_joint_positions(self.local_goal_r, raw=False)
            rospy.sleep(0.01)

        elif self.goal_type == 'trajectory':
            self._left_client.send_goal(self.local_goal_l, feedback_cb=self._feedback)
            self._right_client.send_goal(self.local_goal_r, feedback_cb=self._feedback)

        print("goal sent\n")


    def goal_iteration(self):
        """
        Iterates through the whole list of goals in the trajectory request and
        executes the trajectory in discrete 2 step sequences
        """
        print("Starting goal iteration\n")
        self._l_arm.set_joint_position_speed(1.0)
        self._r_arm.set_joint_position_speed(1.0)
        for idx in range(self.iterations - 1):
            # if self.goal_type == 'trajectory':
            #     result = False
            #         while result == False:
            #             self.make_local_goal(idx)
            #             self.send_goal()
            #             result = self.wait()
            # elif self.goal_type == 'API':
            self.make_local_goal(idx)
            self.send_goal()
            self.current += 1

            #TODO RL agent interface here
            #compute error between desired and current position (do this before or after?)
            # self.compare_world_frame(idx+1)
            self.append_dump()


            # if self.debug:
            #     print("Error between desired and current end effector position: \n")
            #     print(self.world_error[-1])
            #     print("\n")

        print("goal iteration complete\n")

    def compare_world_frame(self, index):
        """
        Compares the absolute error between current end effector position as listened to from gazebo to the
        world frame positions parsed from the original demonstration file
        """
        self.world_error.append([
                abs(self.end_effector_pos[0] - self.end_effector_desired['gazebo0_x'][index]),
                abs(self.end_effector_pos[1] - self.end_effector_desired['gazebo0_y'][index]),
                abs(self.end_effector_pos[2] - self.end_effector_desired['gazebo0_z'][index])
                ])

    def dump_variables(self, dump_filename):
        filename = dump_filename + '.pkl'
        with open(filename, 'w') as f:
            pickle.dump([self.gazebo_dump, self.state_dump, self.action_dump, self.reward_dump, self.world_error], f)




def main():
    """RSDK Joint Trajectory Example: File Playback

    Plays back joint positions honoring timestamps recorded
    via the joint_recorder example.

    Run the joint_recorder.py example first to create a recording
    file for use with this example. Then make sure to start the
    joint_trajectory_action_server before running this example.

    This example will use the joint trajectory action server
    with velocity control to follow the positions and times of
    the recorded motion, accurately replicating movement speed
    necessary to hit each trajectory point on time.
    """
    epilog = """
Related examples:
  joint_recorder.py; joint_position_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        '-f', '--file', metavar='PATH', required=True,
        help='path to input file'
    )
    parser.add_argument(
        '-l', '--loops', type=int, default=1,
        help='number of playback loops. 0=infinite.'
    )
    parser.add_argument(
        '-p', '--pickle', type=str, required=True,
        help='path to save pickled dump file'
    )
    # remove ROS args and filename (sys.arv[0]) for argparse
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rollout_execution")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    traj = RolloutExecuter(debug=False)
    traj.parse_file(path.expanduser(args.file))
    traj.init_state_vector()
    #for safe interrupt handling
    rospy.on_shutdown(traj.stop)
    traj.goal_iteration()

    # result = True
    # loop_cnt = 1
    # loopstr = str(args.loops)
    # if args.loops == 0:
    #     args.loops = float('inf')
    #     loopstr = "forever"
    # while (result == True and loop_cnt <= args.loops
    #        and not rospy.is_shutdown()):
    #     print("Playback loop %d of %s" % (loop_cnt, loopstr,))
    #     traj.start()
    #     result = traj.wait()
    #     traj.publish_bool(0)
    #     loop_cnt = loop_cnt + 1
    print("Dumping variables\n")
    traj.dump_variables(str(args.pickle))
    print("Dumped\n")
    print("Exiting - File Playback Complete")

if __name__ == "__main__":
    main()

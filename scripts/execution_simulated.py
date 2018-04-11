#!/usr/bin/env python
"""
 Inherited from Rethink Robotics' Trajectory class

 Changes:
   1.  Record internal as well as Vicon state every
       record cycle
   2.  Save each demonstration into a pre-defined directory
       post-fixed by date and a given number
   3.  Overall recording is done after recording just the
       internals first and then replaying to get both states
"""

from __future__ import print_function

import argparse
import operator
import sys
import threading

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

class RolloutExecuter(Trajectory):
    '''
    Class inherited from Trajectory, modified to support the following capabilities:
    - load the augmented data from a record file (Vicon positions)
    - rolling out recorded trajectories as a discrete chain of start -> goal movements
    - subscribe to Vicon motion capture topic and compute rewards based on world frame error
    - interfaces with an RL agent class, sending and receiving data
    - computes an additive "action" for the robot to take and sends that to the controllers
    '''

    def __init__(self, debug=False):
        #TODO add initialization args
        super(RolloutExecuter, self).__init__()
        self.debug = debug

        self.gazebo_sub = rospy.Subscriber('gazebo/link_states', gazebo_msgs.LinkStates, self.gazebo_callback)
        self.end_effector_pos = None
        self.end_effector_desired = dict()

        #intermediay list of joint angles that will be popped and sent as goals
        self.r_goal_list = []
        self.l_goal_list = []

        #local trajectory goal
        self.local_goal = FollowJointTrajectoryGoal()

        #publish timing topics
        self.timing_pub_state = rospy.Publisher('/cycle_bool', std_msgs.Bool, queue_size = 1)
        self.timing_msg_state = std_msgs.Bool()
    	self.timing_pub_time = rospy.Publisher('/cycle_time', std_msgs.Time, queue_size = 1)
    	self.timing_msg_time = std_msgs.Time()

    def gazebo_callback(self, data):
        self.end_effector_pos = [data.pose[-1].position.x, data.pose[-1].position.y, data.pose[-1].position.z]

    def publish_bool(self, trajectory_state):
        if trajectory_state:
            self.timing_msg_state.data = 1
        else:
            self.timing_msg_state.data = 0
        self.timing_msg_time = rospy.Time.now()
        self.timing_pub_state.publish(self.timing_msg_state)
        self.timing_pub_time.publish(self.timing_msg_time)

    def _execute_gripper_commands(self):
        start_time = rospy.get_time() - self._trajectory_actual_offset.to_sec()
        r_cmd = self._r_grip.trajectory.points
        l_cmd = self._l_grip.trajectory.points
        pnt_times = [pnt.time_from_start.to_sec() for pnt in r_cmd]
        end_time = pnt_times[-1]
        rate = rospy.Rate(self._gripper_rate)
        now_from_start = rospy.get_time() - start_time
        self.publish_bool(1)
        while(now_from_start < end_time + (1.0 / self._gripper_rate) and
              not rospy.is_shutdown()):
            idx = bisect(pnt_times, now_from_start) - 1
            if self._r_gripper.type() != 'custom':
                self._r_gripper.command_position(r_cmd[idx].positions[0])
            if self._l_gripper.type() != 'custom':
                self._l_gripper.command_position(l_cmd[idx].positions[0])
            rate.sleep()
            now_from_start = rospy.get_time() - start_time

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
        and adds the gazebo states to a list for reward computation

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
            elif 'j' == name[0]:
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
        self.local_goal_r = FollowJointTrajectoryGoal()
        self.local_goal_l = FollowJointTrajectoryGoal()

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
        last_time = self.local_goal_r.trajectory.points[-1].time_from_start.to_sec()
        time_buffer = rospy.get_param(self._param_ns + 'goal_time', 0.0) + 1.5
        timeout = rospy.Duration(self._slow_move_offset +
                                 last_time +
                                 time_buffer)
        if self.debug:
            print("last time:\n")
            print(last_time)
            print("\n")
            print("time_buffer:\n")
            print(time_buffer)
            print("\n")
            print("timeout:\n")
            print(timeout)
            print("\n")

        l_finish = self._left_client.wait_for_result(timeout)
        r_finish = self._right_client.wait_for_result(timeout)
        l_result = (self._left_client.get_result().error_code == 0)
        r_result = (self._right_client.get_result().error_code == 0)

    def send_goal(self):
        """
        Sends FollowJointTrajectoryAction request

        @param r/l_goal_k: input single trajectory goal at k index from _r/l_goal.trajectory.points
        """
        if self.debug:
            print("goals:\n")
            print(self.local_goal_r)
            print("\n")
        self._left_client.send_goal(self.local_goal_l)
        self._right_client.send_goal(self.local_goal_r)
        # Syncronize playback by waiting for the trajectories to start
        print("goal sent\n")
        # while not rospy.is_shutdown() and not self._get_trajectory_flag():
        #     rospy.sleep(0.05)
        #     print("waiting\n")
        # self._execute_gripper_commands()

    def goal_iteration(self):
        """
        Iterates through the whole list of goals in the trajectory request and
        executes the trajectory in discrete 2 step sequences
        """
        print("Starting goal iteration\n")
        for idx in range(len(self._r_goal.trajectory.points)-1):
            result = False
            while result == False:
                self.make_local_goal(idx)
                self.send_goal()
                result = self.wait()
                if idx % 100 == 0:
                    print("100 points\n")
        print("goal iteration complete\n")



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
    # remove ROS args and filename (sys.arv[0]) for argparse
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rollout_execution")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")

    traj = RolloutExecuter(debug=True)
    traj.parse_file(path.expanduser(args.file))
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
    # print("Exiting - File Playback Complete")

if __name__ == "__main__":
    main()

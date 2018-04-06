#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics,
# 2018 Priyam Parashar, Anthony Simeonov
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Custom trajectory playback class which thresholds trajectory commands by
comparing current world state with reference world state from demonstration
trajectory and decides if the current goal should be sent to RLearner as input
to optimize on or not...
"""

from __future__ import print_function
import argparse
import operator
import sys
import threading
from queue import Queue

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
    JointTrajectoryPoint, )

import baxter_interface
import baxter_examples

from baxter_interface import CHECK_VERSION

import std_msgs.msg as std_msgs
import sensor_msgs.msg as sensor_msgs
import vicon.msg as vicon


class ThresholdedTrajectory(baxter_examples.Trajectory):
    '''


    "To learn, or not to learn, that is the question" - Not Shakespeare


    Child of Trajectory class modified to keep track of the error offset between
    the current world state and the reference world state, and invokes RL when
    it exceeds a set threshold.

    Modifications:
        1.  parse_file() method does not populate trajectory goal with all
            waypoints at once, instead stores these goals in a list and then
            main() sends these goals one-by-one to the controllers
        2.  After every execution the main() loop checks the difference between
            world state stored in provided file and the current world state. If
            this 
    '''

    def __init__(self, threshold, action_vector):
        super(ThresholdedTrajectory, self).__init__()
        # intermediate Queues to keep track of parsed waypoints
        self._left_trajectory_goals = Queue()
        self._right_trajectory_goals = Queue()
        # TODO store all vicon joint names in a list
        self.vicon_joint_names = []
        # TODO define vicon subscribers
        # self.vicon_j1_sub = ...
        # self.vicon_j2_sub = ...
        # ...

    def parse_file(self, filename):
        """
        Parses input file into:
            a.  Queues of 'FollowJointTrajectoryGoal's
            b.  Indexed list of corresponding world states

        @param filename: input filename
        """
        #open recorded file
        with open(filename, 'r') as f:
            lines = f.readlines()
        #read joint names specified in file
        joint_names = lines[0].rstrip().split(',')
        #parse joint names for the left and right limbs
        for name in joint_names:
            if name[:-3] == 'left':
                self._l_goal.trajectory.joint_names.append(name)
            elif name[:-3] == 'right':
                self._r_goal.trajectory.joint_names.append(name)

        def find_start_offset(pos):
            #create empty lists
            cur = []
            cmd = []
            dflt_vel = []
            vel_param = self._param_ns + "%s_default_velocity"
            #for all joints find our current and first commanded position
            #reading default velocities from the parameter server if specified
            for name in joint_names:
                if name[:-3] == 'left':
                    cmd.append(pos[name])
                    cur.append(self._l_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
                elif name[:-3] == 'right':
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
            cmd, values = self._clean_line(values, joint_names)
            #find allowable time offset for move to start position
            if idx == 0:
                # Set the initial position to be the current pose.
                # This ensures we move slowly (LOL) to the starting point of the
                # trajectory from the current pose - The user may have moved
                # arm since recording
                cur_cmd = [
                    self._l_arm.joint_angle(jnt)
                    for jnt in self._l_goal.trajectory.joint_names
                ]
                self._add_point(cur_cmd, 'left', 0.0)
                cur_cmd = [
                    self._r_arm.joint_angle(jnt)
                    for jnt in self._r_goal.trajectory.joint_names
                ]
                self._add_point(cur_cmd, 'right', 0.0)
                start_offset = find_start_offset(cmd)
                # Gripper playback won't start until the starting movement's
                # duration has passed, and the actual trajectory playback begins
                self._slow_move_offset = start_offset
                self._trajectory_start_offset = rospy.Duration(
                    start_offset + values[0])
            #add a point for this set of commands with recorded time
            cur_cmd = [cmd[jnt] for jnt in self._l_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'left', values[0] + start_offset)
            cur_cmd = [cmd[jnt] for jnt in self._r_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'right', values[0] + start_offset)
            cur_cmd = [cmd['left_gripper']]
            self._add_point(cur_cmd, 'left_gripper', values[0] + start_offset)
            cur_cmd = [cmd['right_gripper']]
            self._add_point(cur_cmd, 'right_gripper', values[0] + start_offset)

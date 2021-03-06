#!/usr/bin/env python
"""
 Inherited from Rethink Robotics' JointRecorder class

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
import threading
from os import path

from baxter_experiments.joint_trajectory import Trajectory
import baxter_examples
import baxter_interface
import rospy

import geometry_msgs.msg as geometry_msgs

TEMP_FILE = '~/.ros/temp_demo_baxter'


class ViconRecorder(baxter_experiments.JointRecorder):
    '''
    Class inherited from JointRecorder modified to launch record() in a separate
    thread. record() also runs in two separate modes now: the guided by human
    user mode and the autonomous mode where the trajectory is replayed and
    corresponding vicon readings are recorded.
    '''

    def __init__(self, filename, rate):
        super(ViconRecorder, self).__init__(filename, rate)

        self._filename = path.expanduser(TEMP_FILE)
        self._joint_filename = filename

        self.joints_vicon = [['world%d_x' % i, 'world%d_y' % i, 'world%d_z' % i] for i in range(1)]

        #initialize world frame (x, y, z) position of vicon joints
        self.pose_vicon = [[-100, -100, -100]]*len(self.joints_vicon)

        # TODO make vicon subscribers more compact
        self.vicon_sub = rospy.Subscriber('/vicon/marker1/pose', geometry_msgs.PoseStamped, self.vicon_callback)
        # self.vicon_j2 =
        # ...j3
        # ...j4

    def reset(self):
        '''
        Resets the done parameter for consequetive recording sessions
        '''
        self._done = False

    def vicon_callback(self, data):
        """
        Callback function for vicon subscriber, fills attribute of current vicon position
        of end effector
        """
        self.pose_vicon[0] = [data.pose.position.x, data.pose.position.y, data.pose.position.z]


    def record_joint_demo(self):
        '''
        Same as record() method, modified to record both vicon and internal
        states
        '''

        if self._joint_filename:
            joints_left = self._limb_left.joint_names()
            joints_right = self._limb_right.joint_names()
            with open(self._joint_filename, 'w') as f:
                f.write('time,')
                f.write(','.join([j for j in joints_left]) + ',')
                f.write('left_gripper,')
                f.write(','.join([j for j in joints_right]) + ',')
                f.write('right_gripper')  #note comma change
                for i_ind in range(len(self.joints_vicon)):
                    f.write(',' + ','.join([j for j in self.joints_vicon[i_ind]]))
                f.write('\n')

                while not self.done():
                    # Look for gripper button presses
                    if self._io_left_lower.state:
                        self._gripper_left.open()
                    elif self._io_left_upper.state:
                        self._gripper_left.close()
                    if self._io_right_lower.state:
                        self._gripper_right.open()
                    elif self._io_right_upper.state:
                        self._gripper_right.close()
                    angles_left = [
                        self._limb_left.joint_angle(j) for j in joints_left
                    ]
                    angles_right = [
                        self._limb_right.joint_angle(j) for j in joints_right
                    ]


                    f.write("%f," % (self._time_stamp(), ))

                    f.write(','.join([str(x) for x in angles_left]) + ',')
                    f.write(str(self._gripper_left.position()) + ',')

                    f.write(','.join([str(x) for x in angles_right]) + ',')
                    f.write(str(self._gripper_right.position()))  #note comma change

                    for i_ind in range(len(self.joints_vicon)):
                        f.write(',' + ','.join([str(x) for x in self.pose_vicon[i_ind]]))
                    f.write('\n')

                    self._rate.sleep()


class JointDemoRecorder(Trajectory):
    '''
    Class inherited from Trajectory modified for syncing with vicon recorder to
    write vicon states along with internal states observed by the robot
    '''

    def __init__(self, filename, rate, debug=False):
        super(JointDemoRecorder, self).__init__()
        self.recorder = ViconRecorder(filename, rate)

        # #limb interface - current angles needed for start move
        # self._l_arm = baxter_interface.Limb('left')
        # self._r_arm = baxter_interface.Limb('right')

        self.local_goal_r = dict()
        self.local_goal_l = dict()

        self.debug = debug

    def start(self):
        """
        Sends FollowJointTrajectoryAction request
        """
        self._left_client.send_goal(self._l_goal, feedback_cb=self._feedback)
        self._right_client.send_goal(self._r_goal, feedback_cb=self._feedback)
        # Syncronize playback by waiting for the trajectories to start
        while not rospy.is_shutdown() and not self._get_trajectory_flag():
            rospy.sleep(0.05)
        self._execute_gripper_commands()

    def make_local_goal(self, k):
        """
        Makes a local trajectory action requst goal based on
        the k and k+1 indices in the overall goal list

        @parak k: index of start point in the overall list of points
        """
        #make a dictionary for joint angles for API

        self.local_goal_r = dict(zip(
                self._r_goal.trajectory.joint_names,
                self._r_goal.trajectory.points[k].positions))

        self.local_goal_l = dict(zip(
                self._l_goal.trajectory.joint_names,
                self._l_goal.trajectory.points[k].positions))

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

        # self._l_arm.move_to_joint_positions(self.local_goal_l)
        # self._r_arm.move_to_joint_positions(self.local_goal_r)
        self._l_arm.set_joint_positions(self.local_goal_l, raw=False)
        self._r_arm.set_joint_positions(self.local_goal_r, raw=False)
        rospy.sleep(0.025)

    def goal_iteration(self):
        """
        Iterates through the whole list of goals in the trajectory request and
        executes the trajectory in discrete 2 step sequences
        """
        print("Starting goal iteration\n")
        self._l_arm.set_joint_position_speed(0.25)
        self._r_arm.set_joint_position_speed(0.25)

        process = threading.Thread(target=self.recorder.record_joint_demo)
        process.daemon = True
        process.start()

        self.make_local_goal(0)
        self._l_arm.move_to_joint_positions(self.local_goal_l)
        self._r_arm.move_to_joint_positions(self.local_goal_r)

        for idx in range(1,len(self._r_goal.trajectory.points) - 2):
            self.make_local_goal(idx)
            self.send_goal()


def main():
    """
    Policy recording for control optimization using reinforcement
    learning

    Record the policy that user demonstrates, replay it and append records with
    vicon data.
    """
    epilog = """
Related examples:
  joint_position_file_playback.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        formatter_class=arg_fmt, description=main.__doc__, epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f',
        '--file',
        dest='filename',
        required=True,
        help='the file name to record to')
    required.add_argument(
        '-o',
        '--old',
        dest='old_file',
        required=True,
        help='the file name to playback the trajectoy from (old file)'
    )
    parser.add_argument(
        '-r',
        '--record-rate',
        type=int,
        default=100,
        metavar='RECORDRATE',
        help='rate at which to record (default: 100)')

    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("policy_recorder")
    print("Getting robot state... ")
    robot = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
    print("Enabling robot... ")
    robot.enable()

    joint_recorder = JointDemoRecorder(args.filename, args.record_rate)

    # process = threading.Thread(target=joint_recorder.recorder.record)
    # process.daemon = True
    # process.start()
    # raw_input("Recording. Press <Enter> to stop.")
    # joint_recorder.recorder.stop()

    print(
        "\nDone recording. The program will now playback the whole demonstra"
        "tion and record the joint states. Please steer clear of the work area. The"
        " recording will begin in...")
    for i in xrange(5):
        rospy.sleep(1.)
        print("%d seconds..." % (5 - i))
    print("Starting joint recording...")
    joint_recorder.parse_file(path.expanduser(args.old_file))

    # record routine in sync
    joint_recorder.recorder.reset()
    # joint_recorder.start()
    # result = joint_recorder.wait()
    # joint_recorder.recorder.stop()
    joint_recorder.goal_iteration()
    print("Exiting - Demo Recording Complete")


if __name__ == '__main__':
    main()

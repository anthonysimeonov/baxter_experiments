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

import baxter_examples
import baxter_interface
import rospy

TEMP_FILE = '/.ros/temp_demo_baxter'


class ViconRecorder(baxter_examples.JointRecorder):
    '''
    Class inherited from JointRecorder modified to launch record() in a separate
    thread. record() also runs in two separate modes now: the guided by human
    user mode and the autonomous mode where the trajectory is replayed and 
    corresponding vicon readings are recorded.
    '''

    def __init__(self, filename, rate):
        super(ViconRecorder, self).__init__(filename, rate)

        self._filename = TEMP_FILE
        self._joint_filename = filename

        # TODO make a list of vicon joint names
        self.joints_vicon = []
        # TODO define vicon subscribers ( starting with 1D case)
        # self.vicon_j1 =
        # self.vicon_j2 =
        # ...j3
        # ...j4

    def reset(self):
        '''
        Resets the done parameter for consequetive recording sessions
        '''
        self._done = False

    def record_joint_demo(self):
        '''
        Same as record() method, modified to record both vicon and internal
        states
        '''

        if self._joint_filename:
            joints_left = self._limb_left.joint_names()
            joints_right = self._limb_right.joint_names()
            with open(self._filename, 'w') as f:
                f.write('time,')
                f.write(','.join([j for j in joints_left]) + ',')
                f.write('left_gripper,')
                f.write(','.join([j for j in joints_right]) + ',')
                f.write('right_gripper,')
                f.write(','.join([j for j in self.joints_vicon]) + '\n')

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

                    # TODO extract all vicon joint data from vicon subscribers

                    f.write("%f," % (self._time_stamp(), ))

                    f.write(','.join([str(x) for x in angles_left]) + ',')
                    f.write(str(self._gripper_left.position()) + ',')

                    f.write(','.join([str(x) for x in angles_right]) + ',')
                    f.write(str(self._gripper_right.position()) + '\n')

                    # TODO write the extracted data from line number 82 to file

                    self._rate.sleep()


class JointDemoRecorder(baxter_examples.Trajectory):
    '''
    Class inherited from Trajectory modified for syncing with vicon recorder to
    write vicon states along with internal states observed by the robot
    '''

    def __init__(self, filename, rate):
        super(JointDemoRecorder, self).__init__()
        recorder = ViconRecorder(filename, rate)

    def start(self):
        """
        Sends FollowJointTrajectoryAction request
        """
        self._left_client.send_goal(self._l_goal, feedback_cb=self._feedback)
        self._right_client.send_goal(self._r_goal, feedback_cb=self._feedback)
        # Syncronize playback by waiting for the trajectories to start
        while not rospy.is_shutdown() and not self._get_trajectory_flag():
            rospy.sleep(0.05)
        threading.Thread(target=self.recorder.record_joint_demo())
        self._execute_gripper_commands()


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

    threading.Thread(target=joint_recorder.recorder.record())
    raw_input("Recording. Press <Enter> to stop.")
    joint_recorder.recorder.stop()
    joint_recorder.parse_file(path.expanduser(TEMP_FILE))

    print(
        "\nDone recording. The program will now playback the whole demonstra"
        "tion and record the joint states. Please steer clear of the work area. The"
        " recording will begin in...")
    for i in xrange(5):
        rospy.sleep(1.)
        print("{0} seconds...".format(5 - i))
    print("Starting joint recording...")

    # TODO run the joint_recorder's execution routine, and invoke the recorder's
    # record routine in sync
    joint_recorder.recorder.reset()
    result = True
    while (result == True and not rospy.is_shutdown()):
        print("Playback loop %d of %s" % (
            loop_cnt,
            loopstr,
        ))
        joint_recorder.start()
        result = joint_recorder.wait()
    print("Exiting - Demo Recording Complete")


if __name__ == '__main__':
    main()

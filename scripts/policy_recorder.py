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

import argparse

import baxter_examples
import baxter_interface
import rospy


class PolicyRecorder(baxter_examples.JointRecorder):
    '''
    Class inherited from JointRecorder modified for recording vicon states
    along with internal state observed by the robot
    '''

    def __init__(self, filename, rate):
        super(PolicyRecorder, self).__init__(filename, rate)
        self._filename = '/.ros/temp_demo_baxter'
        self._joint_filename = filename

        self.trajectory_executor = baxter_examples.Trajectory()
        # TODO make a list of vicon joint names
        # self.vicon_joint_names = []
        # TODO define vicon subscribers ( starting with 1D case)
        # self.vicon_j1 =
        # self.vicon_j2 =
        # ...j3
        # ...j4

    def joint_joint_recorder(self, header_flag):
        '''
        A record method that records the joint 'joint states', by appending
        known observed joint-space recording with vicon data. This method is
        called every cycle when the arm has finished chasing the original goal
        to read and append data in the original file on the given line.
        '''
        if header_flag:
            # TODO modify the header of the file by appending vicon_joint_names
            # to the list
            pass
        else:
            # TODO read the vicon data, format and append to the intended file
            pass


def main():
    """
    Policy recording for iterative control optimization using reinforcement
    learning

    Record the policy that user demonstrates, replay it and append records with
    vicon data.
    """
    epilog = """
Related examples:
  joint_position_file_playback.py; joint_trajectory_file_playback.py.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-f', '--file', dest='filename', required=True,
        help='the file name to record to'
    )
    parser.add_argument(
        '-r', '--record-rate', type=int, default=100, metavar='RECORDRATE',
        help='rate at which to record (default: 100)'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("policy_recorder")
    print("Getting robot state... ")
    robot = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    robot.enable()

    recorder = PolicyRecorder(args.filename, args.record_rate)
    rospy.on_shutdown(recorder.stop)

    print("Recording. Press Ctrl-C to stop.")
    recorder.record()

    print("\nDone.")


if __name__ == '__main__':
    main()

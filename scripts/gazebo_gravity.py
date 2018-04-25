#!/usr/bin/env python

import sys
import rospy
from gazebo_msgs.srv import *

link_names = ['baxter::head', 'baxter::left_upper_shoulder',
  'baxter::left_lower_shoulder', 'baxter::left_upper_elbow', 'baxter::left_lower_elbow',
  'baxter::left_upper_forearm', 'baxter::left_lower_forearm', 'baxter::left_wrist',
  'baxter::l_gripper_l_finger', 'baxter::l_gripper_r_finger', 'baxter::right_upper_shoulder',
  'baxter::right_lower_shoulder', 'baxter::right_upper_elbow', 'baxter::right_lower_elbow',
  'baxter::right_upper_forearm', 'baxter::right_lower_forearm', 'baxter::right_wrist',
  'baxter::r_gripper_l_finger', 'baxter::r_gripper_r_finger']

def get_properties(link_names):
    rospy.wait_for_service('gazebo/get_link_properties')
    try:
        get_link_properties = rospy.ServiceProxy('gazebo/get_link_properties', GetLinkProperties)
        link_properties = dict()
        for name in link_names:
            response = get_link_properties(name)
            link_properties[name] = response
        return link_properties
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        return ()

def set_properties(link_properties_dict, gravity = False):
    rospy.wait_for_service('gazebo/get_link_properties')
    request = SetLinkProperties()._request_class()
    try:
        set_link_properties = rospy.ServiceProxy('gazebo/set_link_properties', SetLinkProperties)
        for name in link_properties_dict.keys():
            response = link_properties_dict[name]
            request.link_name = name
            request.gravity_mode = gravity
            request.com = response.com
            request.mass = response.mass
            request.ixx = response.ixx
            request.ixy = response.ixy
            request.ixz = response.ixz
            request.iyy = response.iyy
            request.iyz = response.iyz
            request.izz = response.izz
            output = set_link_properties(request)
        return (output)
    except rospy.ServiceException, e:
        print("Service call failed: %s"%e)
        return (False)



def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        if sys.argv[1] in ['false', 'False', 'off'] or sys.argv == 0:
            gravity = False
        else:
            gravity = True
    else:
        print("please specify gravity on/off (true/false)")
        sys.exit(1)
    print ("Setting gravity to %r \n" %gravity)
    properties = get_properties(link_names)
    set_properties(properties, gravity)
    print("Gravity mode set\n")

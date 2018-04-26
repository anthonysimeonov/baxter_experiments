#!/usr/bin/env python

import rospy
import std_msgs.msg as std_msgs
import time

class baxter_suppressions():
    def __init__(self, left=True, right=True, contact=True, collisions=True, body=True, overwrench=True, gravity=True):
        self.left_contact_pub = rospy.Publisher('/robot/limb/left/suppress_contact_safety', std_msgs.Empty, queue_size=1)
        self.right_contact_pub = rospy.Publisher('/robot/limb/right/suppress_contact_safety', std_msgs.Empty, queue_size=1)

        self.left_collisions_pub = rospy.Publisher('/robot/limb/left/suppress_collision_avoidance', std_msgs.Empty, queue_size=1)
        self.right_collisions_pub = rospy.Publisher('/robot/limb/right/suppress_collision_avoidance', std_msgs.Empty, queue_size=1)

        self.left_body_pub = rospy.Publisher('/robot/limb/left/suppress_body_avoidance', std_msgs.Empty, queue_size=1)
        self.right_body_pub = rospy.Publisher('/robot/limb/right/suppress_body_avoidance', std_msgs.Empty, queue_size=1)

        self.left_overwrench_pub = rospy.Publisher('/robot/limb/left/suppress_hand_overwrench_safety', std_msgs.Empty, queue_size=1)
        self.right_overwrench_pub = rospy.Publisher('/robot/limb/right/suppress_hand_overwrench_safety', std_msgs.Empty, queue_size=1)

        self.left_gravity_pub = rospy.Publisher('/robot/limb/left/suppress_gravity_compensation', std_msgs.Empty, queue_size=1)
        self.right_gravity_pub = rospy.Publisher('/robot/limb/right/suppress_gravity_compensation', std_msgs.Empty, queue_size=1)


        self._left = left
        self._right = right

        self.limbs = [self._left, self._right]

        self._contact = contact
        self._collisions = collisions
        self._body = body
        self._overwrench = overwrench
        self._gravity = gravity

        self.suppressions = [self._contact, self._collisions, self._body, self._overwrench, self._gravity]

    def suppress_right(self):
        msg = std_msgs.Empty()
        if self._right:
            if self._contact:
                self.right_contact_pub.publish(msg)
            if self._collisions:
                self.right_collisions_pub.publish(msg)
            if self._body:
                self.right_body_pub.publish(msg)
            if self._overwrench:
                self.right_overwrench_pub.publish(msg)
            if self._gravity:
                self.right_gravity_pub.publish(msg)

    def suppress_left(self):
        msg = std_msgs.Empty()
        if self._left:
            if self._contact:
                self.left_contact_pub.publish(msg)
            if self._collisions:
                self.left_collisions_pub.publish(msg)
            if self._body:
                self.left_body_pub.publish(msg)
            if self._overwrench:
                self.left_overwrench_pub.publish(msg)
            if self._gravity:
                self.left_gravity_pub.publish(msg)                


def main():
    rospy.init_node('baxter_suppressor', anonymous = True)
    rate = rospy.Rate(15)
    suppressions = baxter_suppressions()

    print("Turning on all relevant suppressions for the baxter... stay safe... \n")

    while not rospy.is_shutdown():
        suppressions.suppress_right()
        suppressions.suppress_left()
        rate.sleep()


if __name__ == '__main__':
    main()

#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, Pose, Point, PointStamped

import sys, select, termios, tty

import tf
import tf2_geometry_msgs

import math

def abs(n):
    if n<0:
        return -n
    return n


def turn_to_polaris(ps, pub):
    x = ps.point.x
    y = ps.point.y

    turn = 0.5 #controls turn speed, too high will overshoot and oscilate

    #turn angle
    th = math.atan2(y,x)

    target_turn = 0

    try:

        target_turn = turn * th

        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_turn

        rospy.logdebug("aligning with Polaris")
        rospy.logdebug("THETA: %s"%th)
        pub.publish(twist)

    except:
        pass

if __name__=="__main__":
    """
    this node subscribes to /polaris_base_link_coords and basically tries to nullify the y component of the
    vector pointing towards the polaris projection by publishing Twist() messages on the /cmd_vel_mux/input/teleop topic
    #TODO: figure out why publishing on topics other than /cmd_vel_mux/input/teleop doesnt move the robot 
    """

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('align_with_polaris', log_level=rospy.DEBUG)
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=20)
    sub = rospy.Subscriber('/polaris_base_link_coords', PointStamped, lambda ps: turn_to_polaris(ps,pub))

    rospy.spin()

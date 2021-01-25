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
    
    turn = 1
    th = (math.pi/2 - math.atan2(y,x))
    target_turn = 0

    try:

        target_turn = turn * th

        #if target_turn > control_turn:
        #    control_turn = min( target_turn, control_turn + 0.1 )
        #elif target_turn < control_turn:
        #    control_turn = max( target_turn, control_turn - 0.1 )
        #else:
        #    control_turn = target_turn

        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = target_turn
        rospy.loginfo("aligning with Polaris")
        pub.publish(twist)

        #print("loop: {0}".format(count))
        #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
        #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        pass

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('align_with_polaris')
    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=20)
    sub = rospy.Subscriber('/polaris_base_link_coords', PointStamped, lambda ps: turn_to_polaris(ps,pub))

    rospy.spin()

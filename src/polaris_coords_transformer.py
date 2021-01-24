#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, Pose, Point

import sys, select, termios, tty

import tf

from geometry_msgs.msg import PointStamped


def handle_polaris_position(ps, pub, listener):


    bl_ps = listener.transformPoint('base_link', ps)

    pub.publish(bl_ps)


if __name__=="__main__":


    rospy.init_node('polaris_coords_transformer')
    listener = tf.TransformListener()

    pub = rospy.Publisher('/polaris_base_link_coords', PointStamped, queue_size=5)
    sub = rospy.Subscriber('/polaris_coords', PointStamped, lambda ps:handle_polaris_position(ps,pub,listener))

    rospy.spin()

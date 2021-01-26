#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, Pose, Point

import sys, select, termios, tty

import tf
from tf.transformations import quaternion_matrix, translation_matrix
from geometry_msgs.msg import PointStamped
import numpy as np



def handle_polaris_position(ps, pub, listener):
    (trans,rot) = listener.lookupTransform("/base_link", "/odom", rospy.Time(0));
    rot_m = quaternion_matrix(rot)
    trans_m = translation_matrix(trans)
    t_matrix = np.dot(trans_m, rot_m)
    rospy.logdebug("TRANSLATION: %s"%trans)
    rospy.logdebug("ROTATION: %s"%rot)
    new_p = np.dot(t_matrix,[ps.point.x,ps.point.y,ps.point.z, 1])
    new_p = new_p[:3]
    new_ps = PointStamped(point = Point(new_p[0],new_p[1],new_p[2]))

    new_ps.header.frame_id = 'base_link'
    new_ps.header.stamp = rospy.Time.now()
    pub.publish(new_ps)

if __name__=="__main__":
    """
    this node transforms points from /odom to /base_link and publishes on /polaris_base_link_coords
    """

    rospy.init_node('polaris_coords_transformer', log_level=rospy.INFO)

    listener = tf.TransformListener()
    pub = rospy.Publisher('/polaris_base_link_coords', PointStamped, queue_size=30)
    sub = rospy.Subscriber('/polaris_odom_coords', PointStamped, lambda ps:handle_polaris_position(ps,pub,listener))

    rospy.spin()

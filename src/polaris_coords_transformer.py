#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, Pose, Point

import sys, select, termios, tty

import tf

from geometry_msgs.msg import PointStamped


def handle_polaris_position(ps, pub, listener):
    #bl_ps = listener.transformPoint('base_link',ps)
    #pub.publish(bl_ps)

    (trans,rot) = listener.lookupTransform('earth', 'base_link', rospy.Time(0))
    (dx,dy,dz) = trans
    x = ps.point.x + dx
    y = ps.point.y + dy
    z = ps.point.z + dz
    bl_ps = PointStamped()
    bl_ps.point = Point(x,y,z)
    bl_ps.header.frame_id = 'base_link'
    bl_ps.header.stamp = rospy.Time.now()
    rospy.loginfo(ps)
    rospy.loginfo("TRANSLATION: %s"%trans)
    pub.publish(bl_ps)



if __name__=="__main__":


    rospy.init_node('polaris_coords_transformer')
    listener = tf.TransformListener()

    pub = rospy.Publisher('/polaris_base_link_coords', PointStamped, queue_size=30)
    sub = rospy.Subscriber('/polaris_coords', PointStamped, lambda ps:handle_polaris_position(ps,pub,listener))

    rospy.spin()

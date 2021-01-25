#!/usr/bin/env python  
import tf
import roslib
import rospy
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from numpy import deg2rad

if __name__ == '__main__':

    rospy.init_node('earth_transform_publisher')

    radius    = rospy.get_param("earth_radius")
    latitude  = rospy.get_param("odom_latitude")
    longitude = rospy.get_param("odom_longitude")
    theta     = rospy.get_param("odom_z_rotation")

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(200)

    q = tf.transformations.quaternion_from_euler(0, deg2rad(90-latitude),  deg2rad(longitude))

    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, -radius),
                         q,
                         rospy.Time.now(),
                         "/earth",
                         "/odom")
        rate.sleep()

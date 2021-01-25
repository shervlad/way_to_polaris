#!/usr/bin/env python  
import roslib
import rospy
import tf2_ros
import tf
import tf_conversions
from numpy import deg2rad

if __name__ == '__main__':

    rospy.init_node('earth_transform_publisher')

    radius    = rospy.get_param("earth_radius")
    latitude  = rospy.get_param("odom_latitude")
    longitude = rospy.get_param("odom_longitude")
    theta     = rospy.get_param("odom_z_rotation")

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20)

    q = tf_conversions.transformations.quaternion_from_euler(deg2rad(latitude), deg2rad(longitude), deg2rad(-theta))

    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, -radius),
                         q,
                         rospy.Time.now(),
                         "earth",
                         "odom")
        rate.sleep()

#!/usr/bin/env python  
import roslib
import rospy
import tf2_ros
import tf
import tf_conversions

if __name__ == '__main__':

    rospy.init_node('earth_transform_publisher')

    radius    = rospy.get_param("earth_radius")
    latitude  = rospy.get_param("odom_latitude")
    longitude = rospy.get_param("odom_longitude")
    theta     = rospy.get_param("odom_z_rotation")

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    q = tf_conversions.transformations.quaternion_from_euler(latitude, longitude, -theta)

    while not rospy.is_shutdown():
        br.sendTransform((0.0, 0.0, -1.0*radius),
                         q,
                         rospy.Time.now(),
                         "odom",
                         "earth")
        rate.sleep()

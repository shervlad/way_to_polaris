#!/usr/bin/env python  
import tf
import roslib
import rospy
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
from numpy import deg2rad

if __name__ == '__main__':

    """
    this was a first attempt at defining a tf transform from earth to /odom frame by trying to 
    avoid doing the math in polaris_coords.py and simply define the transform from /odom to earth
    in terms of translation and rotation from /odom to earth and let tf do the inverse and give us earth->/odom
    this proved fruitless and hard to debug and doing the math in polaris_coords.py turtned out to be a more 
    intuitive way for me to transform from earth to /odom frame
    """
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

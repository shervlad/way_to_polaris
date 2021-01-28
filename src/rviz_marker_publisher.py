#!/usr/bin/env python  
import rospy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
from math import atan2
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker
import numpy as np

def publish_marker(ps,pub):

    x,y,z = ps.point.x,ps.point.y,ps.point.z
    v = np.array((x,y,z))
    v = 10*v/np.sqrt(np.sum(v**2))
    points = [Point(0,0,0),Point(*v)]
    m = Marker()
    m.type=m.ARROW
    m.points = points
    m.header.frame_id = 'base_link'
    m.header.stamp = rospy.Time.now()
    m.color.a = 1
    m.scale.x = 0.2
    m.scale.y =1
    m.scale.z =1
    m.id = 0
    pub.publish(m)

if __name__ == '__main__':
    rospy.init_node('rviz_marker_publisher',log_level=rospy.DEBUG)
    pub = rospy.Publisher('/rviz_marker', Marker, queue_size=20)
    sub = rospy.Subscriber('/polaris_base_link_coords', PointStamped, lambda ps:publish_marker(ps,pub))
    rospy.spin()

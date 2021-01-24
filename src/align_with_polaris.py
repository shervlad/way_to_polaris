#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, Pose, Point

import sys, select, termios, tty

import tf
import tf2_geometry_msgs

speed = .2
turn = 1

def transform_pose(input_pose, from_frame, to_frame):

    # **Assuming /tf topic is being broadcasted
    tf_buffer = tf.Buffer()
    listener = tf.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        raise

def handle_polaris_position(p, pub):
    polaris_pose = Pose()
    polaris_pose.x = p.x
    polaris_pose.y = p.y
    polaris_pose.z = p.z

    polaris_base_link_pose = transform_pose(polaris_pose, 'earth','base_link')
    th = -polaris_base_link_pose.x
    target_turn = 0
    control_turn = 0
    try:

        target_turn = turn * th

        if target_turn > control_turn:
            control_turn = min( target_turn, control_turn + 0.1 )
        elif target_turn < control_turn:
            control_turn = max( target_turn, control_turn - 0.1 )
        else:
            control_turn = target_turn

        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
        pub.publish(twist)

        #print("loop: {0}".format(count))
        #print("target: vx: {0}, wz: {1}".format(target_speed, target_turn))
        #print("publihsed: vx: {0}, wz: {1}".format(twist.linear.x, twist.angular.z))

    except:
        print e
    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('align_with_polaris')
    pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)
    sub = rospy.Subscriber('/polaris_coords', Point, handle_polaris_position,pub)

    rospy.spin()

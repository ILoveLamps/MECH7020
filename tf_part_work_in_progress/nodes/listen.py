#!/usr/bin/env python  
import roslib
roslib.load_manifest('ttbot_waypoint')
import rospy
import math
import tf
import geometry_msgs.msg
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('listen')

    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('odom', 'my_marker', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

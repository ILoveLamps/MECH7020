#!/usr/bin/env python  

import roslib
import rospy
import tf
from tf import transformations as ts

from geometry_msgs.msg import Transform, Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
roslib.load_manifest('ttbot_waypoint')

if __name__ == '__main__':
    rospy.init_node('listen')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
       
        try:
            (trans, rot) = listener.lookupTransform('odom', 'tag_5', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
	
        rate.sleep()

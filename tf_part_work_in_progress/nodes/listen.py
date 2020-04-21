#!/usr/bin/env python
"""
    04.20.20
    Listener Node
    - Hirdayesh Shrestha
    - Matthew Postell
    - Dhruv Patel
"""

import roslib
import rospy
import tf

roslib.load_manifest('ttbot_waypoint')

if __name__ == '__main__':

    # ******************************
    #       Main Program Loop
    # ******************************

    rospy.init_node('listen')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('base_footprint', 'my_marker', rospy.Time(), rospy.Duration(5.0))
            (trans, rot) = listener.lookupTransform('base_footprint', 'my_marker', rospy.Time())
            print("Translation :  \n {} \n".format(trans))
            print("Rotation :  \n {} \n".format(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

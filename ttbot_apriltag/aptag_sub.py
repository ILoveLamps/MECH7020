#!/usr/bin/env python
"""
    Detect no of tags, tag ids, and transform tags
    - Hirdayesh Shrestha
    - Matthew Postell
    - Dhruv Patel
"""

import rospy
import tf
import math
import os
from tf.transformations import euler_from_quaternion
from apriltag_ros.msg import AprilTagDetectionArray


#  # GLOBAL VARIABLES
tag_id = []
tag_quantity = 0


def tag_qty_callback(msg):
    """"""
    """ 
        Determines how many tags exist from /tag_detections
        Also stores the tag ids
    """
    """"""
    global tag_id, tag_quantity

    tag_quantity = len(msg.detections)

    if tag_quantity > 0:
        for x in range(tag_quantity):
            tag_id.insert(x, msg.detections[x].id[0])
            # print('tag_{}'.format(tag_id[x]))


def transform():
    """"""
    """ 
        Calculates heading and distance to tag from transformed data
    """
    """"""
    if tag_quantity > 0:
        for x in range(tag_quantity):
            try:
                tag_name = 'tag_' + str(tag_id[x])
                (trans, rot) = listener.lookupTransform('base_footprint', tag_name, rospy.Time())
                distance = math.sqrt(trans[0]**2 + trans[1]**2 + trans[2]**2)

                orientation_list = [rot[0], rot[1], rot[2], rot[3]]
                (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                yaw = math.degrees(yaw)

                print('************************')
                print("Distance to {} :  \n {} \n".format(tag_name, distance))
                print("Heading to {}:  \n {}".format(tag_name, yaw))
                print('************************')
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
    else:
        print('NO TAGS FOUND')


if __name__ == '__main__':
    rospy.init_node('aptag_sub')
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_qty_callback)
    listener = tf.TransformListener()

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        transform()
    r.sleep()


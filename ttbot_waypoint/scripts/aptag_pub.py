#!/usr/bin/env python
"""
    Detect no. of tags, tag ids, and then transform tags
    Publish id, range, bearing, quantity to /aptag_info topic
    - Hirdayesh Shrestha
    - Matthew Postell
    - Dhruv Patel
"""

import rospy
import tf
import math
import numpy as np
from ttbot_waypoint.msg import tag_array
from rospy.numpy_msg import numpy_msg
from apriltag_ros.msg import AprilTagDetectionArray


#  # GLOBAL VARIABLES
# tag_id = []
# tag_quantity = 0


def tag_qty_callback(msg):
    """"""
    """ 
        Determines how many tags exist from /tag_detections
        Also stores the tag ids
    """
    """"""
    # global tag_id, tag_quantity
    tag_id = []
    tag_quantity = len(msg.detections)

    if tag_quantity > 0:
        for x in range(tag_quantity):
            tag_id.insert(x, msg.detections[x].id[0])
            # publishTagData.id.insert(x, tag_id[x])
            # print('tag_{}'.format(tag_id[x]))

    publishTagData.id = tag_id                  # # float64 list of ids
    publishTagData.quantity = tag_quantity      # # int64 number of quantity
    # print(tag_id)


def transform():
    """"""
    """ 
        Calculates heading and distance to tag from transformed data
    """
    """"""
    distance = []
    bearing = []

    if publishTagData.quantity > 0:
        for x in range(publishTagData.quantity):
            try:
                tag_name = 'tag_' + str(publishTagData.id[x])
                (trans, rot) = listener.lookupTransform('base_footprint', tag_name, rospy.Time())
                temp_dist = math.sqrt(trans[0]**2 + trans[1]**2 + trans[2]**2)
                distance.insert(x, temp_dist)

                temp_bearing = math.atan2((trans[1]), (trans[0]))
                temp_bearing = ((180 * temp_bearing) / math.pi)
                bearing.insert(x, temp_bearing)

                # print('************************')
                # print("Distance to {} :  \n {} \n".format(tag_name, distance))
                # print("Heading to {}:  \n {}".format(tag_name, bearing))
                # print('************************')
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
    else:
        pass
        # print('NO TAGS FOUND')

    # print(bearing)
    publishTagData.range = distance
    publishTagData.bearing = bearing

    pub.publish(publishTagData)


if __name__ == '__main__':
    rospy.init_node('aptag_sub')
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_qty_callback)

    pub = rospy.Publisher('/aptag_info', numpy_msg(tag_array), queue_size=10)
    publishTagData = tag_array()

    listener = tf.TransformListener()

    r = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        transform()
    r.sleep()


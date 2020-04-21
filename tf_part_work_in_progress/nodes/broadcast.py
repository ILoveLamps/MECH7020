#!/usr/bin/env python
"""
    04.20.20
    Broadcast Node
        1. Camera Feed Frame (april-tag detection)
        2. TurtleBot3 Frame (odometry) 
    - Hirdayesh Shrestha
    - Matthew Postell
    - Dhruv Patel

"""
import roslib
import rospy
import tf
from tf.transformations import *
import struct
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovariance
from apriltag_ros.msg import AprilTagDetectionArray

from std_msgs.msg import (
    Header,
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Point32,
    Quaternion,
)


roslib.load_manifest('ttbot_waypoint')


def find_marker(detections,marker_num):
    """
    Finds index of marker #marker_num in the apriltag detection message detections. If not found,
    returns -1.
    """
    for i in range(0,len(detections)):
        if detections[i].id==marker_num:
            return i
    return -1


def get_marker(detections):
    """
    Parses an apriltag detection message and gets the index for the last used marker, or
    simply the first one found if the last one was not seen. Returns -1 if nothing found.
    """
    if (len(detections)==0):
        return None,None
    last_marker = -1
    marker = find_marker(detections,last_marker)
    if (marker!=-1):
        return last_marker,marker
    else:
        last_marker=detections[0].id
        return detections[0].id,0




def get_t_R(pose):
    """
    Returns the translation vector (4x1) and rotation matrix (4x4) from a pose message
    """
    t=np.transpose(np.matrix([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,1]))
    quat=[pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    R_full=quaternion_matrix(quat)
    R=R_full
    return t,R

def handle_tag_pose(msg):
    br = tf.TransformBroadcaster()
    # Function to get the pose from msg 
    (my_marker,marker_pos) = get_marker(msg.detections)

    (marker_t, marker_R)= get_t_R(msg.detections[marker_pos].pose.pose)	    

    print("TAG : \n\n", marker_t)		
    br.sendTransform((marker_t[0], marker_t[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, marker_t[2]),
                     rospy.Time.now(),
		             "my_marker",
                     "world")


def handle_bot_pose(msg):
    br = tf.TransformBroadcaster()

    pose = msg.pose.pose	    
    print("BOT : \n\n", pose)		
    br.sendTransform((pose.position.x, pose.position.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, pose.position.z),
                     rospy.Time.now(),
		             "odom",
                     "world")



if __name__ == '__main__':

    # ******************************
    #       Main Program Loop
    # ******************************

    rospy.init_node('broadcast')

    global last_marker
    last_marker = -1
    
    rospy.Subscriber('/tag_detections',
                     AprilTagDetectionArray,
                     handle_tag_pose)
    
    rospy.Subscriber('/odom',
                     Odometry,
                     handle_bot_pose)
	
    rospy.spin()

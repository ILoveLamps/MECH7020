#!/usr/bin/env python
"""
    04.20.20
    Listener Node
    - Hirdayesh Shrestha
    - Matthew Postell
    - Dhruv Patel
"""

# Listener File for getting the transformation of april_tags


import rospy
import tf
import math
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from apriltag_ros.msg import AprilTagDetectionArray




def get_marker(detections):
    """
    Parses an apriltag detection message and gets the index for the last used marker, or
    simply the first one found if the last one was not seen. Returns -1 if nothing found.
    """
    if (len(detections)==0):
        print("\n **** NO LAND MARK FOUND **** \n")
	return None

    else:
	global marker
	l = len(detections)
	marker = np.ones((l,1),dtype=int)	
	for i in range(len(detections)):

            marker[i]=detections[i].id
            
	return marker

def handle_bot_pose(msg):
    pose = msg.pose.pose
   
    global bot_x, bot_y, dy, dx
    bot_x = pose.position.x
    bot_y = pose.position.y
    
    dy = abs( trans[1] ) - abs ( bot_y )
    dx = abs( trans[0] ) - abs ( bot_x )
	
    bearing = abs( np.arctan2(dy,dx) )
    global bearing
    bearing = abs( (180 * bearing ) / math.pi )
    
    


def handle_tag_pose(msg):

    marker = get_marker(msg.detections)
            

    for tag_id in range(len(marker)):
	
	tag_name = 'tag_' + str(marker[tag_id][0])
        # Listener syntax 
        # listener.lookupTransform('to_frame', 'from_frame', rospy.Time())
	global trans
        (trans, rot) = listener.lookupTransform('base_footprint', tag_name, rospy.Time())
            
        orientation_list = [rot[0], rot[1], rot[2], rot[3]]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        distance = math.sqrt(trans[0]**2 + trans[1]**2 + trans[2]**2)
	yaw = abs( (180 * yaw ) / math.pi )
	roll = abs( (180 * roll) / math.pi )
	pitch = abs( (180 * pitch) / math.pi )
	bearing = np.arctan2( (trans[1]), (trans[0]) )
	bearing = ( (180 * bearing ) / math.pi )

	print("Tag-ID : \n  \t\t  {} \n".format(marker[tag_id]))
        print("Distance to tag : \n \t\t {} \n".format(distance))
	print('Heading : \t {} \n'.format(bearing))
	print('Roll : \t\t {} \n'.format(roll))
	print('Pitch : \t {} \n'.format(pitch))
	print('Yaw : \t\t {} \n'.format(yaw))	
        print('***********************')



def main():
    # **************************
    #     Main Program Loop
    # ************************** 
    
    global listener
    rospy.init_node("listener")
    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
	

    while not rospy.is_shutdown():
        try:
            rospy.Subscriber('/tag_detections',
                     AprilTagDetectionArray,
                     handle_tag_pose)



        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()


if __name__ == '__main__':
    try:

        main()
    
    except rospy.ROSInterruptException:
        pass


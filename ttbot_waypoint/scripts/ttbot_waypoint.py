#!/usr/bin/env python
"""
    Waypoint navigation for turtlebot3
    - Hirdayesh Shrestha

"""


import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


def ttbot_move():
    """"""
    """ Calls odo_listen() to read odometry data and publishes to /cmd_vel to change
        the turtlebot3 position and heading
    """
    """"""

    global STATE, e

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('ttbot_waypoint', anonymous=True)
    twist = Twist()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # # LISTENING TO ODOMETER
        odo_listen()

        # # CONTROL ALGORITHM
        e = theta_goal - yaw                # # error in heading
        if STATE == 0:                      # # initial rotation of robot
            if abs(e) > 0.05:
                twist.angular.z = kp*e      # # proportional controller
                print("Target = {},  Current = {}, Error = {}".format(theta_goal, yaw, e))
            else:
                print("\nINITIAL ROTATION COMPLETE")
                twist.angular.z = 0.0
                STATE = 1

        elif STATE == 1:    # moving robot to waypoint
            if dist > 0.05:
                twist.angular.z = kp*e      # # heading correction
                twist.linear.x = 0.1        # # constant linear velocity
                print("\nDISTANCE TO TARGET = {}".format(dist))
            else:
                twist.linear.x = 0.0
                print("\nYOU HAVE REACHED YOUR DESTINATION")
                STATE = 9

        elif STATE == 9:    # do nothing
            pass

        # # PUBLISHING
        pub.publish(twist)
        rate.sleep()


def callback(data):
    """"""
    """ This function is called after odo_listen() is called
        Populates the global variables listed below
    """
    """"""

    global xcd, ycd, roll, pitch, yaw, theta_goal, dist, e

    # # GOAL HEADING
    xcd = data.pose.pose.position.x; ycd = data.pose.pose.position.y
    theta_goal = math.atan2((goal_y - ycd), (goal_x - xcd))

    # # CURRENT HEADING
    orientation_q = data.pose.pose.orientation  # Orientation of turtlebot3 in Quaternion
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    # # DISTANCE TO GOAL
    dist = math.sqrt((xcd - goal_x)**2 + (ycd - goal_y)**2)
    # print(dist)


def odo_listen():
    """"""
    """ Subscribes to the /odom topic from nav_msgs.msg
    """
    """"""

    rospy.Subscriber("/odom", Odometry, callback)


#  # PID CONSTANTS
kp = 0.4                       # proportional gain

#  # TARGET WAYPOINT
goal_x = -3.0; goal_y = -2.0    # goal xy coordinates

#  # GLOBAL VARIABLES AND INITIALIZATION
xcd = 0.0; ycd = 0.0           # current x coordinate
roll = pitch = yaw = 0.0       # yaw is the current heading
theta_goal = 0.0               # target heading
dist = 0.0                     # pythagorean distance to goal
STATE = 0                      # state variable to transition between behaviors
e = 0.0                        # error in heading
odo_listen()                   # initialize the target and current yaw


if __name__ == '__main__':
    """"""
    """ Main program loop
    """
    """"""

    try:
        ttbot_move()
    except rospy.ROSInterruptException:
        pass
    finally:
        print("\n*** ROSNODE STOPPED BY USER ***")

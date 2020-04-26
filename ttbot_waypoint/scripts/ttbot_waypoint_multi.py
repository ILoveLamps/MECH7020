#!/usr/bin/env python
"""
    Multi-waypoint navigation for turtlebot3
    - Hirdayesh Shrestha
    - Matthew Postell
    - Dhruv Patel
"""


import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


def ttbot_move():
    """"""
    """ PID controller for heading
        Inputs: kp, kd, ki are already specified   
                e is already a global var     
        Outputs: angular velocity          
    """
    """"""
    global STATE, e, dist, waypoint, goal_x, goal_y

    # # INITIALIZING NODE AND PUBLISHER
    rospy.init_node('ttbot_waypoint', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    twist = Twist()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        # # CONTROL ALGORITHM
        e = theta_goal - yaw                        # # error in heading

        if STATE == 0:                              # # initial rotation of robot
            if abs(e) > 0.02:
                twist.angular.z = ttbot_pid()
                # print("Target = {},  Current = {}, Error = {}".format(theta_goal, yaw, e))
            else:
                print("\nINITIAL ROTATION COMPLETE")
                twist.angular.z = 0.0
                STATE = 1

        elif STATE == 1:    # moving robot to waypoint
            if dist > 0.05:
                if flag_right or (flag_right and flag_left):
                    # turn left
                    print("TURN LEFT")
                    twist.angular.z = 0.3
                    twist.linear.x = 0.1
                elif flag_left:
                    # turn right
                    print("TURN RIGHT")
                    twist.angular.z = -0.3
                    twist.linear.x = 0.1
                else:
                    # go straight
                    print("GO STRAIGHT")
                    if abs(e) > 0.02:                   # # lower precision to avoid spins
                        twist.angular.z = ttbot_pid()   # # heading correction
                    twist.linear.x = 0.2                # # constant linear velocity
                # print("DISTANCE TO TARGET = {}".format(dist))
            else:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                print("\nYOU HAVE REACHED YOUR WAYPOINT")
                STATE = 9

        elif STATE == 9:
            waypoint = waypoint + 1         # # Increment waypoint counter
            if waypoint > len(gx) - 1:      # # Checking if last waypoint crossed
                print("ALL WAYPOINTS TRAVERSED")
            else:
                STATE = 0                   # # Repeat State logic for new waypoint
                goal_x = gx[waypoint]       # # Change goal waypoint
                goal_y = gy[waypoint]

        # # PUBLISHING
        pub.publish(twist)
        rate.sleep()


def ttbot_pid():
    """"""
    """ PID controller for heading
        Inputs: kp, kd, ki are already specified   
                e is already a global var     
        Outputs: angular velocity          
    """
    """"""

    global old_e, E

    e_dot = e - old_e               # differential error
    E = E + e                       # integral error

    u = kp*e + kd*e_dot + ki*E      # PID algorithm
    old_e = e
    return u


def odo_listen():
    """"""
    """ 
        - Subscribes to the /odom topic from nav_msgs.msg
        - Only needs to be subscribed to once
    """
    """"""
    rospy.Subscriber("/odom", Odometry, callback_odo)


def callback_odo(data):
    """"""
    """ 
        - This function starts after odo_listen() is called
        - Runs continuously as long as long as program is running 
        - Populates the global variables listed below
        - math.atan2 and euler_from_quaternion produces negative angles beyond pi rad
            so angular correction is needed for these values
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


def laser_listen():
    """"""
    """ 
        Subscribes to the /scan topic from sensor_msgs.msg
    """
    """"""
    rospy.Subscriber("/scan", LaserScan, callback_laser)


def callback_laser(data):
    """"""
    """ 
        - This function starts after laser_listen() is called
    """
    """"""
    global flag_left, flag_right, e_obstacle
    obj_limit = 1.0

    # # PORTSIDE OBJECT
    if min(data.ranges[10:50]) < obj_limit:
        flag_left = 1
        e_obstacle = abs(min(data.ranges[10:50]) - obj_limit)
    else:
        flag_left = 0

    # # STARBOARD OBJECT
    if min(data.ranges[310:350]) < obj_limit:
        flag_right = 1
        e_obstacle = abs(min(data.ranges[10:50]) - obj_limit)
    else:
        flag_right = 0


# # GOAL WAYPOINTS
gx = [1.0, 3.0, 1.0, 0.5]
gy = [1.0, 0.5, -2.0, 3.0]

waypoint = 0                    # Waypoint counter starting from 0
goal_x = gx[waypoint]           # Initialize the first waypoint
goal_y = gy[waypoint]

#  # PID GAINS AND VARIABLES
kp = 1.5                        # proportional gain
kd = 0.0                        # differential gain
ki = 0.0                        # integral gain
e = 0.0                         # error in heading
e_obstacle = 0.0                # error in obstacle distance
old_e = 0.0                     # variable to store previous error
E = 0.0                         # integral error

#  # GLOBAL VARIABLES AND INITIALIZATION FOR ODOMETRY
xcd = 0.0; ycd = 0.0            # current x coordinate
roll = pitch = yaw = 0.0        # yaw is the current heading
theta_goal = 0.0                # target heading
dist = 0.0                      # pythagorean distance to goal
STATE = 0                       # state variable to transition between behaviors

#  # GLOBAL VARIABLES AND INITIALIZATION FOR LASER SCAN
flag_left = 0                   # 0 is false
flag_right = 0                  # 0 is false


if __name__ == '__main__':
    """"""
    """ Main program loop
    """
    """"""

    try:
        odo_listen()    # start odometry subscription
        laser_listen()  # start LaserScan subscription
        ttbot_move()    # move turtlebot
    except rospy.ROSInterruptException:
        pass
    finally:
        print("\n*** ROSNODE STOPPED BY USER ***")

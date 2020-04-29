#!/usr/bin/env python

# ************************************** #
#         EKF Localization Node          #
# ************************************** #
#                  by Matthew Postell    #
#                     Hirdayesh Shrestha #
#                     Dhruv Patel        #
# ************************************** #
#  Department of Mechanical Engineering  #
#       University of Cincinnati         #
# *************************************  #

# Importing necessary modules

# Modules Specifically Required for ROS part
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


# Modules Related to python and general programming
import math
import numpy as np



def ekf_move():

    # Utilize the Global variables which we will use in this function

    global x_t, P_t, bot_v, bot_w, bot_x, bot_y, bot_theta


    # Initialzing Node and Publisher
    rospy.init_node('ekf_loc_node', anonymous=True)
    pub = rospy.Publisher('/ekf_loc_node', Twist, queue_size=10)

    # ------------------------
    #  Question : What will we publish to /ekf-loc topic
    # ------------------------

    twist = Twist()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

    # ----------------------
    # Dynamics Part of the EKF
    # ----------------------

        G_t = np.array([    [ 1, 0, -(bot_v/bot_w)*(math.cos(bot_theta)) + (bot_v/bot_w)*(math.cos(bot_theta + bot_w*dt)) ],
                            [ 0, 1, -(bot_v/bot_w)*(math.sin(bot_theta)) + (bot_v/bot_w)*(math.sin(bot_theta + bot_w*dt)) ],
                            [ 0, 0, 1]                                                                          ])

        B_t = np.array([    [ -(bot_v/bot_w)*(math.sin(bot_theta)) + (bot_v/bot_w)*(math.sin(bot_theta + bot_w*dt)) ],
                            [ (bot_v/bot_w)*(math.cos(bot_theta)) - (bot_v/bot_w)*(math.cos(bot_theta + bot_w*dt)) ],
                            [ bot_theta*dt ]                                                                    ])

        V11 = ( -math.sin(bot_theta) + math.sin(bot_theta + bot_w*dt) ) / bot_w
        V12 = ( (bot_v/bot_w**2) * (math.sin(bot_theta) - math.sin(bot_theta + bot_w*dt) ) ) + ( (bot_v/bot_w) * (math.cos(bot_theta + bot_w*dt)) * dt )
        V21 = ( math.cos(bot_theta) - math.cos(bot_theta + bot_w*dt) ) / bot_w
        V22 = ( - (bot_v/bot_w**2) * (math.cos(bot_theta) - math.cos(bot_theta + bot_w*dt) ) ) + ( (bot_v/bot_w) * (math.sin(bot_v + bot_w*dt)) * dt )

        V_t = np.array([    [V11, V12],
                            [V21, V22],
                            [0,    dt]      ])


        x_t = F_t.dot(x_t) + B_t
        P_t = (G_t.dot(P_t).dot(G_t.T)) + (V_t.dot(M_t).dot(V_t.T))
	
        print("\n ******* ******* ********* \n")
        print("Pose of Bot : {}".format(x_t))
	
	twist.linear.x = x_t[0]
	twist.linear.y = x_t[1]
	twist.angular.z = x_t[2]





        # Publishing
        pub.publish(twist)
        rate.sleep()





# ----------------------------
#       Control Listen 
# ----------------------------
def control_listen():

# - Subscribes to the /cmd_vel or /custom_node topic 
# - Needs to be listened every time step
    
    rospy.Subscriber("/cmd_vel", Twist, callback_control)

def callback_control(data):

# - This function starts after control_listen() is called
# - We want to call this every time step to get {v} and {w}

    global bot_v, bot_w 

    bot_v = data.linear.x
    bot_w = data.angular.z

    print("**** The bot command received is \n : {} {}".format(bot_v,bot_w))


# ************************************************


# ----------------------------
#       Odometry Listen 
# ----------------------------
def odom_listen():

# - Subscribes to the /odom topic from nav_msgs.msg
# - Only needs to be subscribed to once

    rospy.Subscriber("/odom", Odometry, callback_odo)

def callback_odo(data):
    
# - This function starts after odom_listen() is called
# - We only want to call this once for the true position and 
#   populate our state space vector ( x, y and theta )

    global bot_x, bot_y, bot_theta, x_t
    # # Getting the x, y and z from the data.pose.pose.position
    bot_x = data.pose.pose.position.x
    bot_y = data.pose.pose.position.y
    bot_z = data.pose.pose.position.z

    # # Getting the roll, pitch and yaw from the data.pose.pose.orientation
    # { Orientation of the turtlebot3 in Quaternion }
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    bot_theta = yaw

    x_t = np.array([ [bot_x], [bot_y], [bot_theta] ])





# *******************************
#   MAIN PROGRAMMING COMMAND
# *******************************

# -------------------------------
#        What are we doing ?
# -------------------------------

"""
1. Prediction Step
      We need x,y,theta for the bot from /odom {once}
              v and w (omega) from the controller from /cmd_vel {every time step}

2. Update Step
      We need range and bearing from the continuous detections topic


"""



# ----------------------------
#       Global Parameters
# ----------------------------

# Global variables and initialization for odometry
bot_x, bot_y, bot_z = 0.0, 0.0, 0.0
bot_theta = 0.0 

# Global variables and initialization for controller
bot_v, bot_w = 0.0, 0.0

# Global variables and initialization for ekf_prediction step
dt = 1/10

F_t = np.array([    [1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]   ])

Q_t = 0.044 * np.identity(3)

M_t = 0.0022 * np.identity(2)

R_t = np.array([    [0.24, 0],
                    [0, 0.24]   ])


x_t = np.array([ [0], [0], [0]])

P_t = 0.66 * np.identity(3)

jump = 0

if __name__ == '__main__':

    try:
	
	if jump == 0:
            # Listen to Odometry to get the state of the bot { X }
            odom_listen()
	    jump = 100
        else:
	    ekf_sub_move()
	    # custom topic listen from ekf
	    # still working on how to read this data from the publisher

        # Listen to CMD_VEL to get the command of movement for the bot { Velocity and Turning Rate }
        control_listen()

        # Carry out EKF once we have the necessary data 
        ekf_move()
    
    except rospy.ROSInterruptException:
        pass

    finally:
        print("\n ***** ROSNODE STOPPED BY USER ***** \n")

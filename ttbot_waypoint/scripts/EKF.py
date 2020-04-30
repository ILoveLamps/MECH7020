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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# # this is the custom message that I created
from ttbot_waypoint.msg import tag_array    
from rospy.numpy_msg import numpy_msg


# Modules Related to python and general programming
import math
import numpy as np

# ************************************************

# ************************************************

def ekf_move():

    # Utilize the Global variables which we will use in this function

    global x_t, P_t, bot_x, bot_y, bot_theta, first_bot_x, first_bot_y, first_bot_theta, bot_v, bot_w, jump, \
        Z, L, len_data, bot_z, roll, pitch, yaw, \
	F_t, B_t, G_t, M_t, V_t

    # Initialzing Node and Publisher
    rospy.init_node('EKF', anonymous=True)
    pub = rospy.Publisher('/ekf_loc', Odometry, queue_size=10)

    # ------------------------
    #  Question : What will we publish to /ekf-loc topic
    # ------------------------

    twist = Odometry()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

    # ----------------------
    # Dynamics Part of the EKF
    # ----------------------
	
#        if jump == 0:
#            x_t = np.array([  [first_bot_x], [first_bot_y], [first_bot_theta]  ])


#        else:
#            x_t = x_t
	    



#        jump = 100

	print("Initial Pose :: Before Motion\n\n", x_t)

        G_t = np.array([    [ 1, 0, -(bot_v/bot_w)*(math.cos(x_t[2])) + (bot_v/bot_w)*(math.cos(x_t[2] + bot_w*dt)) ],
                            [ 0, 1, -(bot_v/bot_w)*(math.sin(x_t[2])) + (bot_v/bot_w)*(math.sin(x_t[2] + bot_w*dt)) ],
                            [ 0, 0, 1]                                                                          ])
	print(G_t)

        B_t = np.array([    [ -(bot_v/bot_w)*(math.sin(x_t[2])) + (bot_v/bot_w)*(math.sin(x_t[2] + bot_w*dt)) ],
                            [ (bot_v/bot_w)*(math.cos(x_t[2])) - (bot_v/bot_w)*(math.cos(x_t[2] + bot_w*dt)) ],
                            [ bot_w*dt ]                                                                    ])
	print("******",B_t)

        V11 = ( -math.sin(x_t[2]) + math.sin(x_t[2] + bot_w*dt) ) / bot_w
        V12 = ( (bot_v/bot_w**2) * (math.sin(x_t[2]) - math.sin(x_t[2] + bot_w*dt) ) ) + ( (bot_v/bot_w) * (math.cos(x_t[2] + bot_w*dt)) * dt )
        V21 = ( math.cos(bot_theta) - math.cos(bot_theta + bot_w*dt) ) / bot_w
        V22 = ( - (bot_v/bot_w**2) * (math.cos(x_t[2]) - math.cos(x_t[2] + bot_w*dt) ) ) + ( (bot_v/bot_w) * (math.sin(x_t[2] + bot_w*dt)) * dt )

        V_t = np.array([    [V11, V12],
                            [V21, V22],
                            [0,    dt]      ], dtype=float)

        x_t = x_t.reshape((3,-1))
        x_t = F_t.dot(x_t) + B_t
        P_t = (G_t.dot(P_t).dot(G_t.T)) + (V_t.dot(M_t).dot(V_t.T))


        print("\n ******* ******* \n")
        print("Motion :: Pose of Bot : \n {}".format(x_t))


	
	    # ----------------------
        # Measurement Part of the EKF
        # ----------------------

        # Number of Tags we read at any step 
        # { Determines the length of the for loop }


        for i in range(len_data):
        

            
            g = math.sqrt( (L[0][i] - x_t[0])**2 + (L[1][i] - x_t[1])**2 )
            h = ( np.arctan2( L[1][i] - x_t[1], L[0][i] - x_t[0] ) ) - x_t[2]
                
            z = np.array([g, h, Z[2][i]])

            # Reshaping the measurement {Z} and the update model {z}

            z = z.reshape(3,-1)
            Measure = Z[:,i].reshape((3,-1))
	    print("\n\n **************** \n {}".format(Measure))
	    print("\n\n **************** \n {}".format(z))		
            innovation = Measure - z


            # Measurement Matrix
            h11 = -(L[0][i]- x_t[0]) / g
            h12 = -(L[1][i] - x_t[1]) / g
            h21 = (L[1][i] - x_t[1]) / (g**2)
            h22 = -(L[0][i] - x_t[0]) / (g**2)
                
            H_t = np.array([    [h11, h12, 0],
                                [h21, h22, -1],
                                [0, 0, 0]       ], dtype=float)
                
            S = np.linalg.inv((H_t.dot(P_t).dot(H_t.T)) + R_t)
                
            K = P_t.dot(H_t.T).dot(S)
        
            x_t = x_t + K.dot( innovation ) 
            P_t = P_t - K.dot(H_t).dot(P_t)

        print("\n ******* ******* \n")
       	print("Update :: Pose of Bot : \n {}".format(x_t))
	



        twist.pose.pose.position.x = x_t[0]
	twist.pose.pose.position.y = x_t[1]


	yaw = x_t[2]
	
	q_back = quaternion_from_euler(roll, pitch, yaw)
	print(q_back)
	twist.pose.pose.orientation.x = q_back[0]
	twist.pose.pose.orientation.y = q_back[1]
	twist.pose.pose.orientation.z = q_back[2]
	twist.pose.pose.orientation.w = q_back[3]
	
	x_t = x_t
	P_t = P_t

        # Publishing
        pub.publish(twist)
        rate.sleep()





# ************************************************


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
#         World Listen 
# ----------------------------
def world_listen():

# - Listens to the custom_msg we get from the /tag_detections
# - This custom msg is already tranformed from the camera frame ...
#   ... to the base frame of the robot

    rospy.Subscriber('/aptag_info', numpy_msg(tag_array), callback_world)

def callback_world(data):

    """
        Here we will prepare the data for measurement update.
        We need two things for the update step. 
            1. Location data : already defined as global variables
            2. Measurement data
        We have to run the "for loop" in the EKF Update step for {n}
        number of times ... where {n = total number of landmarks in the frame}
    """
    global Z, len_data

    # This the measurement data we are getting from the continuous detection
    len_data = data.quantity
    Z = np.ones((3,len_data))
    for xx in range(len_data):
	Z[0,xx] = data.x[xx]
	Z[1,xx] = data.y[xx]
	Z[2,xx] = data.id[xx]

	


# ************************************************



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
first_bot_x, first_bot_y = 0.0, 0.0
yaw = 0.0
bot_z = 0.0
roll = 0.0
pitch = 0.0 
first_bot_theta = yaw

# Global varibles and initialization for prediction part
bot_x, bot_y, bot_theta = 0.0, 0.0, 0.0

# Global variables and initialization for controller
bot_v, bot_w = 0.001, 1e-5

# Global variables and initialization for ekf_prediction step
dt = 0.033*3

F_t = np.array([    [1, 0, 0],
                    [0, 1, 0],
                    [0, 0, 1]   ])

Q_t = 0.044 * np.identity(3)

M_t = 0.0022 * np.identity(2)

R_t = np.array([    [0.24, 0, 0],
                    [0, 0.24, 0],
                    [0, 0, 0.24]   ])


x_t = np.array([ [first_bot_x], [first_bot_y], [first_bot_theta]])

P_t = 0.66 * np.identity(3)

# Global variables and initialization for ekf_update step
Z = []


# Defining Exact Tag Location { knowledge we have for localization }
# - Each column will represent one tag with ...
# ... 1st Row : x - Position of Tag
# ... 2nd Row : y - Position of Tag
# ... 3rd Row : Tag id

L = np.array([  [6.919942, 6.074146, -1.112591, -0.710367],
                [0.007912, -6.064760, -5.972452, 1.518057],
                [1, 4, 8, 12]   ])


jump = 0



# ************************************************

# ************************************************


if __name__ == '__main__':

    try:
        # Listen to CMD_VEL to get the command of movement for the bot { Velocity and Turning Rate }
        control_listen()

        # Listen to transformed listener to get the data for update step { Range, Bearing and Tag id }
        world_listen()

        # Carry out EKF once we have the necessary data 
        ekf_move()
        

    except rospy.ROSInterruptException:
        pass

    finally:
        print("\n ***** ROSNODE STOPPED BY USER ***** \n")

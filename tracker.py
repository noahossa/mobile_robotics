#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys
import random
import time
from math import atan2, sqrt, pi
import numpy as np
from turtlesim.srv import TeleportAbsolute, SetPen

rospy.init_node('trajectory_description', anonymous=False)

traj_description = rospy.get_param('/tracker/trajectory_description')
xd = traj_description['xd']
yd = traj_description['yd']
n  = traj_description['repeat']
dt = traj_description['timestep']

#get initial position
x_i = rospy.get_param('/tracker/x')
y_i = rospy.get_param('/tracker/y')
w_i = rospy.get_param('/tracker/w')

#Preferable gains according to Siegwart CH3
k_p = 3
k_a = 8
k_b = -1.5

def pose_callback(posedata):

    global x, y, theta
    x = posedata.x
    y = posedata.y
    theta = posedata.theta

def tracker():
    # Creates a node with name 'turtlebot_controller' and make sure it is a
    # unique node (using anonymous=True).
    #rospy.init_node('tracker', anonymous=False)

    # Publisher which will publish to the topic '/turtle1/cmd_vel'.
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)

    # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
    # when a message of type Pose is received.
    
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    rospy.wait_for_service('/turtle1/teleport_absolute')
    teleport_service = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
    teleport_service(x_i, y_i, w_i)

    rospy.wait_for_service('/turtle1/set_pen')
    set_pen = rospy.ServiceProxy('turtle1/set_pen',SetPen)
    



    rate = rospy.Rate(1/dt)          #rate at which the while loop will run
    vel = Twist()                    #define Twist variable composed of 3 linear and 3 angular components from cmd_vel

    count = 0
    laps = 0
    
    rospy.sleep(1)  #small delay to make sure we have a pose
    while not rospy.is_shutdown():
        
        if count >= len(xd):                  #check if we have iterated though all of the provided trajectory points
            
            rospy.loginfo("Finished loop %f", laps+1)  #if so then we completed a lap
            count = 0                                #reset desired points to initial to reset controller to next lap
            laps = laps + 1                          #iterate lap count

        elif laps == n:                              #check if we have reached end of laps

            rospy.loginfo("Finished laps")
            break  

        #define controller variables
        dx    = xd[count] - x         #define difference between actual and desired x and y
        dy    = yd[count] - y


        #debugging:
        rospy.loginfo("Actual x =%f", x)
        rospy.loginfo("Actual y =%f", y)
        #rospy.loginfo("Desired x =%f", xd[0])
        #rospy.loginfo("Desired y =%f", yd[0])
        #rospy.loginfo(theta)

        #Kelly Equations 3.50-3.52
        rho   = sqrt(pow(dx,2) + pow(dy,2))
        alpha = -theta + atan2(dy,dx)
        beta  = -theta - alpha

        e_p = rho
        color = int(255 - (255* np.tanh(5*e_p/(11*sqrt(2)))))
        print(color)
        set_pen(0, color, color, 4, 0)
       
        #adjust alpha and beta to (-pi, pi) range
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        beta  = (beta  + np.pi) % (2 * np.pi) - np.pi

        #define the control law
        v = k_p*rho
        w = k_a*alpha + k_b*beta

        #v = min(v, 5.0)

        #set Twist velocities according to the controller outputs
        vel.linear.x  = v
        vel.linear.y  = 0
        vel.linear.z  = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = w   

        #rospy.loginfo(v)
        #rospy.loginfo(w)
        #count how many trajectory points have been fed
        count = count + 1

        velocity_publisher.publish(vel)
        rate.sleep()  #sleeps for approp time to loop at set rate
        
        
        
if __name__ == '__main__':
    try:
        tracker()
    except rospy.ROSInterruptException:
        pass
# Course: ECED 3901 Engineering Design II
# 
# Authors: Larissa Lemoine, Behnam Nazari, James Hillaby
# This program is written for the ECED 3901 class robo heist. 
# The program is written for Group 14.Members: Larissa Lemoine,
# Behnam Nazari,James Hillaby, Lyndsay Robar
# Robot Name: Pascal
# Date: March 12, 2025
#
# Competition Date: March 20,2025
#
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

# http://docs.ros.org/en/noetic/api/geometry_msgs/html/index-msg.html
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan

from collections import deque
import math
import time
#import class from other file
from eced3901_dt1.py import LaserDataInterface

#New class based on the Navigate square class in eced3901_dt1.py file.
#potential functions/other things to add
# - turn 90(added)
# - turn other 90, potentially merge with above with entering negative/positive value
# - compare lidar
# - drive forward(distance specific? Time specific?)
# - check map cells function
# - turn fully around
# - back up?
# - waypoint array(txt file read in maybe)
# - map (txt read in, array)
# 
class NavigateCourse(Node):
     #initialization taken and edited from navigate square
    def __init__(self):
        #This calls the initilization function of the base Node class
        super().__init__('navigate_square')
        # Ensure these are obviously floating point numbers and not
        # integers (python is loosely typed)

        #Velocity negative for forward driving velocity, backwards in simulation
        self.x_vel = -0.2 # velocity changed to correct value for robot not simulation

        self.x_now = 0.0
        self.x_init = 0.0
        self.y_now = 0.0
        self.y_init = 0.0
        self.d_now = 0.0
        self.d_aim = 1.0
        self.i = 0

        self.laser_range = None

        # Subscribe to odometry
        self.sub_odom = self.create_subscription(
            Odometry, #Matches the import at the top (nav_msgs.msg import Odometry)
            'odom', #odom topic
            self.odom_callback, #Call this function
            10) #Queue size 10

        # Publish to velocity
        self.pub_vel = self.create_publisher(
            Twist, #Expects the twist message format
            'cmd_vel',
            10) #Queue size 10

        # Subscribe to lidar - this requires a policy, otherwise you won't see any data
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.sub_lidar = self.create_subscription(
            LaserScan,
            'scan',
            self.range_callback,
            qos_policy
        )

        self.ldi = LaserDataInterface(logger=self.get_logger())

        self.timer = self.create_timer(0.1, self.timer_callback)
    #function created by James
    def hard_left_turn(self):
        msg = Twist()
        msg.angular.z = 1.0 # sets angular velocity to 1 for a 90 degree turn
        # !!! Potentially edit so that pascal stops...
        msg.linear.x = self.x_vel # keeps linear velocity and creates a larger turn radius
        self.pub_vel.publish(msg) # Publishes data to motors so pascal doesnt stop moving
        start_time = time.time() #initializes a time variable
        while time.time() - start_time < 0.13:  # Keep turning for 0.13 seconds using the time variable
            self.pub_vel.publish(msg) #continues to publish data for the duration of the code to prevent a stop error
            time.sleep(0.1) # sleeps for 0.1s to avoid conflicting commands
    


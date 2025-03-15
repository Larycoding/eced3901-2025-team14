# Course: ECED 3901 Engineering Design II
# 
# Authors: Larissa Lemoine, Behnam Nazari, James Hillaby
# This program is written for the ECED 3901 class robo heist demos. 
# The program is written for Group 14. Members: Larissa Lemoine,
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


#For each demo different movements are needed. Rough outline here
#  - safe - drive forward about 6cm or until lidar is 6cm smaller
#  - QR - Stay in one place, print out the code recieved
#  - Coins - drive straight dont run into wall
#  - Cage - drive forward until coin is picked up, back up
#  - Reed - Drive straight and then rotate
#  - Laser beam - Drive forward certain time or until lidar value
#    And turn around and drive back until other lidar value
#  - RFID - Drive forward drive back, use lidar values?
#  - Drive - from start to safe, turn towards
#  - Drive - from start to Indianna Jones, turn towards
#  - Drive - from start to cage, turn toard it

def noneIsInfinite(value):
    if value is None:
        return math.inf
    else:
        return value

def min_ignore_None(data):
    if not data:
        return math.inf
    return min(data, key=noneIsInfinite)

class Demo(Node):
    def __init__(self):
        #This calls the initilization function of the base Node class
        super().__init__('navigate_square')

        # We can either create variables here by declaring them
        # as part of the 'self' class, or we could declare them
        # earlier as part of the class itself. I find it easier
        # to declare them here

        # Ensure these are obviously floating point numbers and not
        # integers (python is loosely typed)

        # WARNING: Check for updates, note this is set and will run backwards
        #          on the physical model but correctly in simulation.
        self.x_vel = -0.2 # velocity changed to correct value for robot not simulation

        self.x_now = 0.0
        self.x_init = 0.0
        self.y_now = 0.0
        self.y_init = 0.0
        self.d_now = 0.0
        self.d_aim = 1.0
        self.type = 0

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
    def control_example_odom(self):
        """ Control example using odomentry """

        msg = Twist()
        # This has two fields:
        msg.linear.x
        msg.angular.z
		        	
		# Calculate distance travelled from initial
        self.d_now = pow( pow(self.x_now - self.x_init + 0.3, 2) + pow(self.y_now - self.y_init, 2), 0.5 )

        if self.d_now < self.d_aim:
            #msg.linear.x = self.x_vel
            #msg.angular.z = 0.0     
            return       
        else:
            msg.linear.x = 0.0 # //double(rand())/double(RAND_MAX); //fun
            msg.angular.z = 0.0 # //2*double(rand())/double(RAND_MAX) - 1; //fun

        self.pub_vel.publish(msg)
        self.get_logger().info("Sent: " + str(msg))
    def turn_around(self):
        msg = Twist()
        msg.angular.z = 1.0 # sets angular velocity to 1 for a 180 degree turn
        self.pub_vel.publish(msg) # Publishes data to motors so pascal doesnt stop moving
        start_time = time.time()  #initializes a time variable
        while time.time() - start_time < 0.25:  # Keep turning for 0.25 seconds
            self.pub_vel.publish(msg) #continues to publish data for the duration of the code to prevent a stop error
            time.sleep(0.1)    # sleeps for 0.1s to avoid conflicting commands
    def control_example_lidar(self):
        """ Control example using LIDAR"""
        msg = Twist()
        # This has two fields:
        # msg.linear.x
        # msg.angular.z	 	       	
        #either here until after decisions or this fuction call needs to be repeated in a while or for loop.
        laser_rangesA = self.ldi.get_range_array(-21.0)#asks for specific array data..forward plus a few degrees
        #may need to reduce the range because it may be too far back with the offset
        #Note lidar was offset by approximately 21 degrees degree ranges were adjusted for this offset
        laser_rangesB = self.ldi.get_range_array(24.0) #range from 40 degrees to 50 degrees on the left side
        laser_rangesC = self.ldi.get_range_array(69.0) #range from 85 degrees to 95 degrees on the left side
        laser_rangesD = self.ldi.get_range_array(114.0) #range from 130 to 140 degrees left (was added for lab 1 to detect 
        laser_rangesE = self.ldi.get_range_array(84.0) #range from 100 to 110 degreees to the left hand side
        laser_rangesF = self.ldi.get_range_array(54.0) #range from 70 to 80 degrees to the left
        laser_rangesG = self.ldi.get_range_array(159.0) #range from 170 to 180 degrees so back

        #lidar apears to be 21 degrees off center (not sure how to fix  it but quick fix is shifting 21 degrees)

        minHorizDistance = 0.4 #closest distance the side of the robot should get to the box (updated from testing)
        minDiagDistance = 0.5# the max distance when going around a corner...(updated from testing)
	    

        # This gets the minimum range, but ignores NONE values. The LIDAR data isn't always
        # reliable, so we might want to ignore NONEs. We also might want to select the minimum
        # range from our entire sweep.
        Front = min_ignore_None(laser_rangesA) #or []#gets smallest of read in lidar data at 0 degrees
        laserrangesBmin = min_ignore_None(laser_rangesB) #or []#get smallest value from readings around 45 degrees
        laser_ranges_minC = min_ignore_None(laser_rangesC) #or []#get smallest value from reading around 90 degrees
        laser_ranges_minD = min_ignore_None(laser_rangesD) #or [] #get smallest value from reading around 135 degrees
        laser_ranges_minE = min_ignore_None(laser_rangesE) #or [] #get smallest value from reading around 105 degrees
        laser_ranges_minF = min_ignore_None(laser_rangesF) #or [] #get smallest value from reading around 85 degrees
        Back = min_ignore_None(laser_rangesG) # get the smallest value from reading around 180 degrees
        if self.type is 'safe':
            #drive forward until lidar is correct value
            #forward driving for safe cracker
            if Front > 0.16:
                self.x_vel = 0.1
            else:
                self.x_vel = 0
        elif self.type is 'coins':
            #drive forward until wall
            #portion for collecting coins demo
            if Front > 0.22:
                self.x_vel = 0.3
            else:
                self.x_vel = 0 
            #do a third range to slow down
        elif self.type is 'cage':
            #collecting coin from cage, !!!may need adjustment
            #for now drive forward for passive collection system
            if Front > 0.20:
                self.x_vel = 0.1
            else:
                self.x_vel = 0
        elif self.type is 'reed':
            if Front > 0.20:
                self.x_vel = -0.1
            else:
                self.x_vel = 0
        elif self.type is 'rfid':
            if Front > 0.20:
                self.x_vel = 0.1
            elif Front <= 0.20:
                self.x_vel = 0
                self.turn_around()
                self.x_vel = 0.1
            elif Back < 0.40 and Back > 0.20:
                self.x_vel = 0
        elif self.type is 'laser beam':
            # drive forward then drive back
            if Front < 0.3:
                self.x_vel = 0.1
            else:
                self.x_vel = 0
    

            


    def timer_callback(self):
        """Timer callback for 10Hz control loop"""

        #self.get_logger().info(f'Timer hit')

        #self.control_example_odom()
        
        self.control_example_lidar()  #switched to lidar from lab instruction, keep odom commented out unless using


    def odom_callback(self, msg):
        """Callback on 'odom' subscription"""
        #self.get_logger().info('Msg Data: "%s"' % msg)        
        self.x_now = msg.pose.pose.position.x
        self.y_now = msg.pose.pose.position.y

    def range_callback(self, msg):
        """Callback on 'range' subscription"""
        #self.get_logger().info('Scan Data: "%s"' % msg)
        #self.get_logger().info('Scan Data: "%s"' % msg.angle_increment)

        anglestep = msg.angle_increment #amount of angle increments
        self.minrange = msg.range_min #minimum acceptable data
        maxrange = msg.range_max #maximum acceptable data
        ranges = msg.ranges[1:]

        self.laser_range = max(ranges[0:5])
       # self.get_logger().info('Scan Data: "%d"' % len(ranges))

        self.ldi.process_laser_msg(msg)

    def destroy_node(self):
        msg = Twist()
        self.pub_vel.publish(msg)
        super().destroy_node()
        
  
def main(args=None):
    rclpy.init(args=args)
    Demotest = Demo()
    rclpy.spin(Demotest)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

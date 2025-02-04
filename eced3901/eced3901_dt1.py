# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

class LaserDataInterface(object):

    def __init__(self, storage_depth=4, logger=None): 
        self.laser_data = deque()
        self.depth = storage_depth
        self.logger = logger

    def get_logger(self):
        if self.logger:
            return self.logger

    def get_range_array(self, center_deg, left_offset_deg=-5, right_offset_deg=+5, invalid_data=None):
        """
        This function should return an array of lidar data. Set center_deg (0 = front), the array starts at left_offset_deg
        and ends at right_offset_deg. You can choose to select specific points within the offset for example.

        WARNING: THIS FUNCTION IS NOT TESTED AND MAY NOT WORK AS INTENDED. NOT SUGGESTED TO USE AS-IS.
        """

        if center_deg < -180 or center_deg > 180:
            raise ValueError("Invalid range: %d"%center_deg)

        if left_offset_deg < -180 or left_offset_deg > 0:
            raise ValueError("Invalid left offset: %d"%left_offset_deg)

        if right_offset_deg > 180 or right_offset_deg < 0:
            raise ValueError("Invalid right offset: %d"%right_offset_deg)

        # No data yet
        if len(self.laser_data) == 0:
            return None

        angleset_deg = (self.anglestep * 180.0) / 3.14159
        offset_i = round(center_deg / angleset_deg)
        offset_pos = round(right_offset_deg / angleset_deg)
        offset_neg = round(left_offset_deg / angleset_deg)

        #Get absolute values, which may be negative!
        start = offset_i + offset_neg
        end = offset_i + offset_pos
        
        # Remap to -180 to 0 range
        if start > 180:
            start = start - 360

        if end > 180:
            end = end - 360

        # Remap to 180 to 0 range
        if start < -180:
            start = start + 360
        
        if end < -180:
            end = end + 360

        tempdata = self.laser_data[0]
        data = list(map(lambda x: None if x < self.minrange or x > self.maxrange else x, tempdata))
        
        #self.get_logger().info('Scan Data: "%s"' % str(data))
                
        # Index 0 maps to 0, Index len/2 maps to 180, Index len maps to -180
        # Remap this so we have array as expected from -180 to 180
        zero_offset = int(len(data) / 2)
        new_slice = data[zero_offset:] + data[:(zero_offset-1)]

        # Uncomment this to see scan data in console (chatty)
        self.get_logger().info('Scan Data: "%d"' % len(new_slice))
        
        # Normal - we just take a slice
        start_index = round(start / angleset_deg)
        end_index = round(end / angleset_deg)

        start_index += zero_offset
        end_index += zero_offset

        if end_index > len(data):
            end_index = len(data)-1
            raise ValueError("Oops?!")
        
        if start_index < 0:
            start_index = 0
            raise ValueError("Oops?!")

        if end > start:
            lidar_data = new_slice[start_index:end_index]
            lidar_data = lidar_data[::-1]
            #self.get_logger().info('Scan Data: "%d:%d"' % (start_index, end_index))
            self.get_logger().info('Scan Data: "%s"' % str(lidar_data))
            return lidar_data
        else:
            raise NotImplementedError("Function cannot deal with splitting array (typically 180 / -180)")


    def process_laser_msg(self, msg):
        self.anglestep = msg.angle_increment
        self.minrange = msg.range_min
        self.maxrange = msg.range_max
        ranges = msg.ranges[0:]

        # Index 0 is front of robot & counts clockwise from there
        num_points = len(ranges)

        self.laser_data.append(msg.ranges)

        # Get rid of old data
        if len(self.laser_data) > self.depth:
            self.laser_data.popleft()


def noneIsInfinite(value):
    if value is None:
        return float("inf")
    else:
        return value

def min_ignore_None(data):
    return min(data, key=noneIsInfinite)

class NavigateSquare(Node):
    """Simple class designed to navigate a square"""
	

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
        # msg.linear.x
        # msg.angular.z
		        	
		# Calculate distance travelled from initial
        self.d_now = pow( pow(self.x_now - self.x_init, 2) + pow(self.y_now - self.y_init, 2), 0.5 )

        if self.d_now < self.d_aim:
            msg.linear.x = self.x_vel
            msg.angular.z = 0.0            
        else:
            msg.linear.x = 0.0 # //double(rand())/double(RAND_MAX); //fun
            msg.angular.z = 0.0 # //2*double(rand())/double(RAND_MAX) - 1; //fun

        self.pub_vel.publish(msg)
        self.get_logger().info("Sent: " + str(msg))    

    def control_example_lidar(self):
        """ Control example using LIDAR"""
        msg = Twist()
        # This has two fields:
        # msg.linear.x
        # msg.angular.z	 	       	
        #either here until after decisions or this fuction call needs to be repeated in a while or for loop.
        laser_rangesA = self.ldi.get_range_array(0.0)#asks for specific array data..forward plus a few degrees
        #may need to reduce the range because it may be too far back with the offset
        laser_rangesB = self.ldi.get_range_array(45.0) #range from 40 degrees to 50 degrees on the left side
        laser_rangesC = self.ldi.get_range_array(90.0) #range from 85 degrees to 95 degrees on the left side
        laser_rangesD = self.ldi.get_range_array(135.0) #range from 130 to 140 degrees left (was added for lab 1 to detect 

        minHorizDistance = 0.5 #closest distance the side of the robot should get to the box
        minDiagDistance = 0.7 # the max distance when going around a corner...
	    

        #The following check feels like there is an issue because if any of these are none it leaves the function and won't continue
        #for the other things and also should not be an if because the later conditions are only tested if the first is not true.
        #if laser_rangesA is None:
           # self.get_logger().warning("Invalid range data, skipping, see if solves itself...")
            #return
        #if laser_rangesB is None:
            #self.get_logger().warning("Invalid range data, skipping, see if solves itself...")
            #return
        #if laser_rangesC is None:
            #self.get_logger().warning("Invalid range data, skipping, see if solves itself...")
            #return
        #if laser_rangesD is None:
            #self.get_logger().warning("Invalid range data, skipping, see if solves itself...")
            #return

        # This gets the minimum range, but ignores NONE values. The LIDAR data isn't always
        # reliable, so we might want to ignore NONEs. We also might want to select the minimum
        # range from our entire sweep.
        laser_ranges_minA = min_ignore_None(laser_rangesA) #gets smallest of read in lidar data at 0 degrees
        laser_ranges_minB = min_ignore_None(laser_rangesB)#get smallest value from readings around 45 degrees
        laser_ranges_minC = min_ignore_None(laser_rangesC)#get smallest value from reading around 90 degrees
        laser_ranges_minD = min_ignore_None(laser_rangesD)#get smallest value from reading around 135 degrees
      
        front = laser_ranges_minA
        frontleft = laser_ranges_minB
        left = laser_ranges_minC
        backleft = laser_ranges_minD

        #all four ranges passed through above to get minimum reading

        # If ALL the lidar returns are NONE, it means all returns were invalid (probably too close).
        # So only do something if the 
        #decisions to be edited

        #Refer to external sheet for logic breakdown by situation
        # or None statements added to 286,288,291,293,296,299


        if laser_rangesA and laser_rangesB and laser_rangesC and laser_rangesD is None: 
            msg.linear.x = self.x_vel

        else:

            if (front > minHorizDistance) or (front == None): #if front is uncovered / modified by Behnam 
                if (left > minHorizDistance) or (left == None): #if left side doesn't sense the box
                    if backleft > minDiagDistance or (backleft == None):#if back left is bigger than the minimum distance diagonally
                        msg.linear.x = self.x_vel #move forward
                    #elif (frontleft>minDiagDistance or frontleft == None) and backleft <= minDiagDistance:# uncommented by behnam
                        #check the turn condition
                        #rclpy.shutdown() #shutdown code at end/ uncomented by behnam
                    elif backleft <minDiagDistance and frontleft < minDiagDistance :# if front is clear and the front left and back left close to box
                        msg.linear.x = self.x_vel #drive forward
                    elif backleft < minHorizDistance and (frontleft > minDiagDistance or frontleft == None) and (left > minDiagDistance or left == None): # front is clear and back has a box 
                        #Added additional condition to detect box to the left of robot [James] #slow down
                        msg.linear.x = self.x_vel * 0.5 #drive forward but slowly if possible 
                    elif backleft > minHorizDistance and backleft <minDiagDistance and (frontleft>minDiagDistance or frontleft == None):#if the back is within the diagonal distance
                        msg.angular.z = 1.0 # turn around the corner
                elif (left < minHorizDistance): # left distance less than the minimum distance /modified by Behnam
                    if (frontleft > minDiagDistance or frontleft == None) and backleft < minDiagDistance:
                        #rotate left
                        msg.angular.z = 1.0
                    elif (backleft > minHorizDistance or None) and frontleft < minDiagDistance:
                        #rotate right
                        msg.angular.z = -1.0
            elif (front < minHorizDistance) or (front == None): #modified by Behnam
                if float(left) < float(backleft): #if turned toward
                    msg.angular.z = -1.0 #turn right



		#if laser_ranges_minB <0.5 and laser_ranges_minC < 0.5:
                #drive forward
                	#msg.linear.x = self.x_vel
            	#elif laser_ranges_minB > 0.5 and laser_ranges_minC>0.5:
                #if nothing in front or beside it on the left drive forward
                	#msg.linear.x = self.x_vel
            	#elif laser_ranges_minB > 0.5 and laser_ranges_minC <0.5:
                #turn
                	#msg.angular.z = 1.0
            	#elif laser_ranges_minB <0.5 and laser_ranges_minC >0.5:
                #drive forward
                	#msg.linear.x = self.x_vel
        	#elif laser_ranges_minA < 0.5:
            #stop and turn?
            	#if laser_ranges_minB<0.5 and laser_ranges_minC<0.5:
                	#msg.angular.z = 1.0 #turn maybe negative to turn other way


        # Old if statements
       # if laser_ranges_min and laser_ranges_min > 0.5: 
        #    msg.linear.x = self.x_vel
       # elif laser_ranges_min and laser_ranges_min < 0.5:
         #   msg.angular.z = 1.0 #edit potentially for 90 degree turn or turn until certain value using decision

        self.pub_vel.publish(msg)
        self.get_logger().info("Sent: " + str(msg))      

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

    navigate_square = NavigateSquare()
    while True:
        rclpy.spin(navigate_square)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
        navigate_square.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

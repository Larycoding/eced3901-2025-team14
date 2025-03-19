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
import time
import numpy as np

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
        return data
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
        return math.inf
    else:
        return value

def min_ignore_None(data):
    if not data:
        return math.inf
    return min(data, key=noneIsInfinite)




class NavigateSquare(Node):
    """Simple class designed to navigate a square"""
	

    def __init__(self):
        #This calls the initilization function of the base Node class
        super().__init__('navigate_square')

        # Ensure these are obviously floating point numbers and not
        # integers (python is loosely typed)

        # WARNING: Check for updates, note this is set and will run backwards
        #          on the phys[
        self.x_vel = -0.2 # velocity changed to correct value for robot not simulation

        self.x_now = 0.0
        self.x_init = 0.0
        self.y_now = 0.0
        self.y_init = 0.0
        self.d_now = 0.0
        self.d_aim = 1.0
        self.n = 0.0

        self.z_now = 0.0
        self.omega_now = 0.0

        self.yaw_now = 0.0
        self.yaw_target = 0.0

        self.type = "laserbeam"

        self.laser_range = None

        pi = math.pi

        """
        waypoints = [
        [0,0,0],
        [0, 1.07, -pi/2],
        [0.45, 1.07, -pi/2],
        [0.4, 1.07, 0],
        [.4, 1.47, pi/2],
        [.90, 1.47, 0],
        [.90, 1.74, pi/2],
        [1.30, 1.74, pi]
        [1.30, 1.47, pi/2],
        [.9, 1.47, 0],
        [.9, 0, pi],
        [.9, .42, pi/2],
        [1.47, .42, -pi/2],
        [.9, .42, 0],
        [.9, 1.47, -pi/2],
        [.4, 1.47, pi/2],
        [.4, 0, pi]
        ]
        """


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

    def yaw(self):
        topyaw = 2*(self.omega_now*self.z_now)
        bottomyaw = 1 - 2*(self.z_now*self.z_now)
        yaw = np.arctan2(topyaw, bottomyaw)
        return yaw


    def control_example_odom(self):
        """ Control example using odomentry """

        msg = Twist()
        # This has two fields:
        msg.linear.x
        msg.angular.z
        self.yaw_now = self.yaw()	
		# Calculate distance travelled from initial
        self.d_now = pow( pow(self.x_now - self.x_init + 0.3, 2) + pow(self.y_now - self.y_init, 2), 0.5 )

        """
        if self.d_now < self.d_aim:
            msg.linear.x = self.x_vel
            msg.angular.z = 0.0     
            return       
        else:
            msg.linear.x = 0.0 # //double(rand())/double(RAND_MAX); //fun
            msg.angular.z = 0.0 # //2*double(rand())/double(RAND_MAX) - 1; //fun
        """
        if self.n < 40.0:
            msg.linear.x = self.x_vel
        else:
            msg.linear.x = 0.0

        '''
        start_time = time.time()
        if self.n < 4.0:
            msg.angular.z = (-1.0)
            msg.linear.x =0.0
            while time.time() - start_time < 1.0:  # Keep turning for 0.1 seconds
                self.pub_vel.publish(msg)
                time.sleep(0.1)
            self.n = self.n+1.0
            msg.angular.z = 0.0
            msg.linear.x = (-1.0)
            while time.time() - start_time < 2.0:  # Keep turning for 0.1 seconds
                self.pub_vel.publish(msg)
                time.sleep(0.1)
            self.n = self.n + 1
            msg.angular.z = 1.0
            msg.linear.x = 0.0
            while time.time() - start_time < 1.0:  # Keep turning for 0.1 seconds
                self.pub_vel.publish(msg)
                time.sleep(0.1)
            self.n = self.n + 1
            msg.angular.z = 0.0
            msg.linear.x =0.0
            while time.time() - start_time < 1.0:  # Keep turning for 0.1 seconds
                self.pub_vel.publish(msg)
            self.n = self.n+1.0
             '''
        """
        elif self.n >10.0 and self.n < 20.0:
            msg.linear.z = self.x_vel
            msg.angular.x = 0.0
            self.n = self.n+1
            self.pub_vel.publish(msg)
        elif self.n <20.0 and self.n >30.0:
            msg.angular.z = 1.0
            msg.linear.x =0.0
            self.n =self.n+1
            self.pub_vel.publish(msg)
        elif self.n > 30.0:
            msg.angular.z =0.0
            msg.linear.x = 0.0
            self.pub_vel.publish(msg)
            """
        self.pub_vel.publish(msg)
        self.get_logger().info("Sent: " + str(msg))  

    def hard_left_turn(self):
        msg = Twist()
        self.yaw_target = math.pi/2
        if self.yaw_now < self.yaw_target:
            msg.angular.z = 1.0 # sets angular velocity to 1 for a 90 degree turn
        else:
            msg.angular.z =0.0 #stop 
        msg.linear.x = 0.0 # keeps linear velocity and creates a larger turn radius
        self.pub_vel.publish(msg) # Publishes data to motors so pascal doesnt stop moving
        start_time = time.time() #initializes a time variable
        while time.time() - start_time < 0.1:  # Keep turning for 0.1 seconds
            self.pub_vel.publish(msg) #continues to publish data for the duration of the code to prevent a stop error
            time.sleep(0.1)   # sleeps for 0.1s to avoid conflicting commands

        
    def micro_left(self):
        msg = Twist()
        msg.angular.z = 0.2 # sets angular velocity to 1 for a slight left turn
        msg.linear.x = self.x_vel * 0.1 # keeps linear velocity and creates a larger turn radius
        self.pub_vel.publish(msg) # Publishes data to motors so pascal doesnt stop moving
        start_time = time.time() #initializes a time variable
        while time.time() - start_time < 0.1:  # Keep turning for 0.1 seconds
            self.pub_vel.publish(msg) #continues to publish data for the duration of the code to prevent a stop error
            time.sleep(0.1)  # sleeps for 0.1s to avoid conflicting commands

    def micro_right(self):
        msg = Twist()
        msg.angular.z = -0.2 # sets angular velocity to 1 for a slight right turn
        msg.linear.x = self.x_vel * 0.1 # keeps linear velocity and creates a larger turn radius
        self.pub_vel.publish(msg) # Publishes data to motors so pascal doesnt stop moving
        start_time = time.time()  #initializes a time variable
        while time.time() - start_time < 0.1:  # Keep turning for 0.1 seconds
            self.pub_vel.publish(msg) #continues to publish data for the duration of the code to prevent a stop error
            time.sleep(0.1)   # sleeps for 0.1s to avoid conflicting commands

    def turn_around(self):
        msg = Twist()
        self.yaw_target = math.pi
        if self.yaw_now < self.yaw_target:
            msg.angular.z = 1.0 # sets angular velocity to 1 for a 90 degree turn
        else:
            msg.angular.z =0.0 #stop 
        msg.linear.x = 0.0 # keeps linear velocity and creates a larger turn radius
        self.pub_vel.publish(msg) # Publishes data to motors so pascal doesnt stop moving
        start_time = time.time() #initializes a time variable
        while time.time() - start_time < 0.1:  # Keep turning for 0.1 seconds
            self.pub_vel.publish(msg) #continues to publish data for the duration of the code to prevent a stop error
            time.sleep(0.1)   # sleeps for 0.1s to avoid conflicting commands
            
    def drive_straight(self):
        msg = Twist()
        msg.angular.z = 0.0 #no angular velocity
        msg.linear.x = self.x_vel #drive forward
        start_time = time.time() #initializes a time variable
        self.get_logger().info(str(msg.linear.x))
        while time.time() - start_time < 0.1:  # Keep turning for 0.1 seconds
            self.pub_vel.publish(msg) #continues to publish data for the duration of the code to prevent a stop error
            time.sleep(0.1)   # sleeps for 0.1s to avoid conflicting commands
    
        
    #function to stop moving
    def stop_moving(self):
        msg = Twist()
        msg.angular.z = 0.0 #no angular velocity
        msg.linear.x = 0.0 #no movement
        self.pub_vel.publish(msg)
        time.sleep(0.1)
    #function to drive backwards
    def drive_back(self):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = self.x_vel*(-1.0)
        self.pub_vel.publish(msg)
        

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
	    

        #The following check feels like there is an issue because if any of these are none it leaves the function and won't continue
        #for the other things and also should not be an if because the later conditions are only tested if the first is not true.

        #Kept for potential future use

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
        laser_ranges_minA = min_ignore_None(laser_rangesA) #or []#gets smallest of read in lidar data at 0 degrees
        laser_ranges_minB = min_ignore_None(laser_rangesB) #or []#get smallest value from readings around 45 degrees
        laser_ranges_minC = min_ignore_None(laser_rangesC) #or []#get smallest value from reading around 90 degrees
        laser_ranges_minD = min_ignore_None(laser_rangesD) #or [] #get smallest value from reading around 135 degrees
        laser_ranges_minE = min_ignore_None(laser_rangesE) #or [] #get smallest value from reading around 105 degrees
        laser_ranges_minF = min_ignore_None(laser_rangesF) #or [] #get smallest value from reading around 85 degrees

        Front = laser_ranges_minA
        frontleft = laser_ranges_minB
        left = laser_ranges_minC
        backleft = laser_ranges_minD
        backleftleft = laser_ranges_minE
        frontleftleft = laser_ranges_minF
        


        #all four ranges passed through above to get minimum reading

        # If ALL the lidar returns are NONE, it means all returns were invalid (probably too close).
        # So only do something if the 
        #decisions to be edited

        #Refer to external sheet for logic breakdown by situation
        # or None statements added to 286,288,291,293,296,299 # code has drastically changed since this
        #self.control_example_odom(self)

        Back = min_ignore_None(laser_rangesG) # get the smallest value from reading around 180 degrees
        """
        if self.type =="safe":
            #drive forward until lidar is correct value
            #forward driving for safe cracker
            

            if Front > 0.22:
                self.drive_straight()
            else:
                self.stop_moving()

        elif self.type == "coins":
            #drive forward until wall
            #portion for collecting coins demo
            

            if Front > 0.22:
                self.drive_straight()
            else:
                self.stop_moving()
            #do a third range to slow down
        elif self.type == "cage":
            #collecting coin from cage, !!!may need adjustment
            #for now drive forward for passive collection system
            if Front > 0.22:
                self.drive_straight()
            else:
                self.stop_moving()
        elif self.type == "reed":
            if Front > 0.23:
                self.drive_straight()
            else:
                self.turn_around()
                self.stop_moving()
        elif self.type == "rfid":
            if Front > 0.22:
                 self.drive_straight()
            elif Front <= 0.22:
                self.stop_moving()
                self.turn_around()
                self.drive_straight()
            elif Back < 0.40 and Back > 0.20:
                self.stop_moving()
        elif self.type == "laserbeam":
            # drive forward then drive back
            if Front > 0.4:
                self.drive_straight()
            else:
                self.stop_moving()

        elif self.type == "run1":

            """
     
        """


        if laser_rangesA and laser_rangesB and laser_rangesC and laser_rangesD is None: # catch statement to aid in filtering out None values 
            msg.linear.x = self.x_vel # if all values are None go forward

        else:  # All the print commands are for use in the console to leave a record of what sections of code are triggering. useful for adjustments

            if ((frontleft) > (minHorizDistance)) or (frontleft == None): #if frontleft is uncovered / modified by Behnam 
                print("1")
                if ((left) > minHorizDistance) or (left == None): #if left side doesn't sense the box
                    print("2")
                    if (backleft) > minDiagDistance or (backleft == None):#if back left is bigger than the minimum distance diagonally
                        msg.linear.x = self.x_vel #move forward
                        print("3")

                    elif (left) > minDiagDistance: #if pascal has exceeded the maximum range of 0.5 turn to the left to close distance
                        print("16")
                        self.micro_left() 

                    elif ((frontleft) > minDiagDistance or frontleft == None) and (backleft) < minDiagDistance:
                        
                        self.micro_left() #micro adjust with left hand turn
                        print("4")

                    elif (backleft) <minDiagDistance and (frontleft) < minDiagDistance :# if front is clear and the front left and back left close to box
                        msg.linear.x = self.x_vel #drive forward
                        print("6")

                    elif backleft < minHorizDistance and (frontleft > minDiagDistance or frontleft == None) and ((left) > minDiagDistance or left == None): # front is clear and back has a box 
                        #Added additional condition to detect box to the left of robot [James] #slow down
                        msg.linear.x = self.x_vel  #drive forward but slowly if possible 
                        print("7")

                    #elif backleft > minHorizDistance and backleft <minDiagDistance and (frontleft>minDiagDistance or frontleft == None):#if the back is within the diagonal distance
                    elif ((left) > minDiagDistance or left == None) and ((frontleft)>minDiagDistance or frontleft == None): # if the pascal has enough space to the 
                        #left and he satifies the conditions for a left hand turn it will complete the entire 90 degree turn at once 
                        self.hard_left_turn()
                        print("8")

                    else:
                        print("error 2") #for troubleshooting errors


                elif ((left) < minHorizDistance): # left distance less than the minimum distance /modified by Behnam
                    print("9")

                    if (frontleft < minDiagDistance): #If the front left ie(45 deg) is within 0.4 use the folowing code to navigate along the wall
                    

                        if (frontleft) == (backleft): #if both the front left ie(45 deg) and backleft ie(135) are equal pascalw ill continue forwards
                            print("10")
                            msg.linear.x = self.x_vel


                        if (frontleft) > (backleft) : #if the front left ie(45 deg) is greater than backleft ie(135 deg) pascal will adjust left
                            print("11")
                            self.micro_left()


                        elif (frontleft) < (backleft) or backleft == None: #if the front left ie(45 deg) is less than backleft ie(135 deg) pascal will adjust right
                            print("12")
                            self.micro_right()

                    else: # if pascal is approaching the corner and the front left ie(45 deg) is a None value or a large value it will trigger the use of 
                        #frontleftleft ie (105 deg) and backleftleft ie(75 deg)


                        if (frontleftleft) == (backleftleft):#if both the front left ie(45 deg) and backleft ie(135) are equal pascalw ill continue forwards
                            print("21")
                        
                            msg.linear.x = self.x_vel


                        if (frontleftleft) > (backleftleft) : #if the front left ie(45 deg) is greater than backleft ie(135 deg) pascal will adjust left
                            self.micro_left()
                            print("22")


                        elif (frontleftleft) < (backleftleft) or backleftleft == None: #if the front left ie(45 deg) is less than backleft ie(135 deg) pascal will adjust right
                            print("23")
                            self.micro_right()
                    

            elif (float(frontleft) < minHorizDistance) :#or (front == None): #modified by Behnam
                print("13")
                self.micro_right()
                #if float(left) < float(backleft): #if turned toward
                 #   print("14")
                  #  msg.angular.z = -1.0 #turn right

            elif (float(front) < minHorizDistance):
                print("wall")
                self.turn_around()

            else:
                print ("error 1")
            """

        self.pub_vel.publish(msg)
        self.get_logger().info("Sent: " + str(msg))      



    def timer_callback(self):
        """Timer callback for 10Hz control loop"""

        #self.get_logger().info(f'Timer hit')

        self.control_example_odom()
        
        #self.control_example_lidar()  #switched to lidar from lab instruction, keep odom commented out unless using
        
        self.get_logger().info("===========: " + str(self.y_now))  
        """
        if self.y_now < -0.2:
            self.i =1
        if self.i and self.y_now > -0.1:
            self.x_vel = 0
            rclpy.shutdown()
         """

    def odom_callback(self, msg):
        """Callback on 'odom' subscription"""
        #self.get_logger().info('Msg Data: "%s"' % msg)        
        self.x_now = msg.pose.pose.position.x
        self.y_now = msg.pose.pose.position.y
        self.z_now = msg.pose.pose.orientation.z
        self.omega_now = msg.pose.pose.orientation.w
        self.yaw_now = self.yaw()	

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
    rclpy.spin(navigate_square)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigate_square.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

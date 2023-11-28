#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math
import time

class MazeSolver:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.front_distance = None
        self.right_distance = None
        self.twist = Twist()
        self.turn_left = False
        self.turn_right = False
        self.turn_right_wall = False
        self.move_forward = False
        self.target_yaw_angle = None

    def odom_callback(self, msg):
        if not self.turn_left and not self.turn_right and not self.turn_right_wall:
            orientation_quaternion = msg.pose.pose.orientation
            orientation_euler = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])
            yaw_angle = orientation_euler[2]

            self.target_yaw_angle_left = (yaw_angle + 1.5708) # 1.5708 radians is approximately 90 degrees
            if self.target_yaw_angle_left > 3:
                self.target_yaw_angle_left = 3.14 
            elif self.target_yaw_angle_left > 1 and self.target_yaw_angle_left < 2:
                self.target_yaw_angle_left = math.pi/2
            elif self.target_yaw_angle_left > 2 and self.target_yaw_angle_left < 2.6:
                self.target_yaw_angle_left = 3*math.pi/4
            elif self.target_yaw_angle_left > -0.1 and self.target_yaw_angle_left < 0.1:
                self.target_yaw_angle_left = 0
            elif self.target_yaw_angle_left > 3:
                self.target_yaw_angle_left = 3.14 
            elif self.target_yaw_angle_left < -1 and self.target_yaw_angle_left > -2:
                self.target_yaw_angle_left = -1 * math.pi/2
            elif self.target_yaw_angle_left < -2 and self.target_yaw_angle_left > -3:
                self.target_yaw_angle_left = -3*math.pi/4
                

            # elif self.target_yaw_angle_left :
            self.target_yaw_angle_right = (yaw_angle - 1.5708) # 1.5708 radians is approximately 90 degrees
            if self.target_yaw_angle_right < -3.14:
                self.target_yaw_angle_right = math.pi/2
            elif self.target_yaw_angle_right > -2.2 and self.target_yaw_angle_right < -1:
                self.target_yaw_angle_right = -1*math.pi/2
            elif self.target_yaw_angle_right > -0.5 and self.target_yaw_angle_right < 0.5:
                self.target_yaw_angle_right = 0
            elif self.target_yaw_angle_right > 3.14:
                self.target_yaw_angle_right = math.pi
            elif self.target_yaw_angle_right > -1 and self.target_yaw_angle_right < 1:
                self.target_yaw_angle_right = 0
            elif self.target_yaw_angle_right > 1 and self.target_yaw_angle_right < 2:
                self.target_yaw_angle_right = math.pi/2
            elif self.target_yaw_angle_right < -2 and self.target_yaw_angle_right > -3:
                self.target_yaw_angle_right= -3*math.pi/4
            
        # moving forward
        if self.move_forward:
            self.twist.linear.x = 0.2
            if self.right_distance < 0.6:
                    self.twist.angular.z = 0.2 * (0.4 - self.right_distance)
            elif self.left_distance < 0.6:
                    self.twist.angular.z = -0.2 * (0.4 - self.left_distance)

        # turning left
        elif self.turn_left and not rospy.is_shutdown():
            orientation_quaternion = msg.pose.pose.orientation
            orientation_euler = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])
            yaw_angle = orientation_euler[2]
            twist_cmd = Twist()
            angular_tolerance = 0.05 # Set a tolerance for stopping rotation
            print(self.target_yaw_angle_left)
            if abs(self.target_yaw_angle_left - yaw_angle) > angular_tolerance:
                # Calculate angular velocity based on the difference between current and target yaw angles
                twist_cmd.angular.z = 0.5 # * (self.target_yaw_angle_left - yaw_angle)
                self.pub.publish(twist_cmd)
            else:
                print("1 turn completed")
                self.pub.publish(Twist())  # Stop the robot
                self.turn_left = False

        # turning right      
        elif self.turn_right and not rospy.is_shutdown():
            print("Turning Right")
            orientation_quaternion = msg.pose.pose.orientation
            orientation_euler = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])
            yaw_angle = orientation_euler[2]
            twist_cmd = Twist()
            angular_tolerance = 0.05 # Set a tolerance for stopping rotation
            if abs(self.target_yaw_angle_right - yaw_angle) > angular_tolerance:
                # Calculate angular velocity based on the difference between current and target yaw angles
                print("target yaw: ", self.target_yaw_angle_right)
                print("yaw: ", yaw_angle)
                twist_cmd.angular.z = -0.3 # * (self.target_yaw_angle_right - yaw_angle)
                self.pub.publish(twist_cmd)
            else:
                print("1 turn completed")
                self.pub.publish(Twist())
                # Set the duration to move forward in seconds
                rate = rospy.Rate(10)  # 10 Hz

                # Drive forward for 3 seconds
                start_time = time.time()
                while time.time() - start_time < 4.5 and not rospy.is_shutdown():
                    twist_cmd = Twist()
                    twist_cmd.linear.x = 0.2  # Adjust linear velocity as needed
                    self.pub.publish(twist_cmd)
                    self.rate.sleep()

                # Stop the robot
                twist_cmd = Twist()
                self.pub.publish(twist_cmd)
                self.turn_right = False

    def scan_callback(self, scan_msg):
        
        if not self.turn_left and not self.turn_right and not self.turn_right_wall:
            self.right_distance = scan_msg.ranges[270]
            self.left_distance = scan_msg.ranges[90]
            back_distance = scan_msg.ranges[180] 
            self.front_distance = scan_msg.ranges[0]
            print("right: ", self.right_distance)
            print("front: ", self.front_distance)


            # when a wall is detected in front and on the right --- turn left
            if self.front_distance < 0.6 and self.right_distance < 1 and not rospy.is_shutdown():
                self.front_distance = scan_msg.ranges[0]
                self.twist.linear.x = 0.0
                self.move_forward = False
                print("turning left")
                self.turn_left = True
                print("front distance: ", self.front_distance)
                print("right distance: ", self.right_distance)
                print(self.turn_left)

            # when a wall not on the right -- turn right
            elif self.right_distance > 1 and not rospy.is_shutdown():
                self.front_distance = scan_msg.ranges[0]

                # Set the duration to move forward in seconds
                rate = rospy.Rate(10)  # 10 Hz

                # Drive forward for 3 seconds
                start_time = time.time()
                while time.time() - start_time < 2 and not rospy.is_shutdown():
                    twist_cmd = Twist()
                    twist_cmd.linear.x = 0.2  # Adjust linear velocity as needed
                    self.pub.publish(twist_cmd)
                    self.rate.sleep()

                # Stop the robot
                twist_cmd = Twist()
                self.pub.publish(twist_cmd)
                self.move_forward = False
                self.turn_right = True
    
                return

            # when a wall to the right and no wall in front --- move forward
            elif self.front_distance > 0.4 and self.right_distance < 0.8 and not rospy.is_shutdown():
                self.front_distance = scan_msg.ranges[0]
                self.move_forward = True

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        maze_solver = MazeSolver()
        maze_solver.run()
    except rospy.ROSInterruptException:
        pass
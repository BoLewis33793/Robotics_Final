#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import math

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
        self.move_forward = False
        self.target_yaw_angle = None

    def odom_callback(self, msg):
        if not self.turn_left and not self.turn_right:
            orientation_quaternion = msg.pose.pose.orientation
            orientation_euler = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])
            yaw_angle = orientation_euler[2]
            self.target_yaw_angle_left = (yaw_angle + 1.5708) # 1.5708 radians is approximately 90 degrees
            if self.target_yaw_angle_left > 3.14:
                self.target_yaw_angle_left = 3.14
            self.target_yaw_angle_right = (yaw_angle - 1.5708) # 1.5708 radians is approximately 90 degrees
            if self.target_yaw_angle_right < -3.14:
                self.target_yaw_angle_right = 1.5708

        if self.move_forward:
            self.twist.linear.x = 0.2

            print("Right Distance: ", self.right_distance)
            print("Left Distance: ", self.left_distance)

            if self.right_distance < 0.4:
                self.twist.angular.z = 0.1
            elif self.left_distance < 0.4:
                self.twist.angular.z = -0.1
            else:
                self.twist.angular.z = 0.0

            # self.twist.angular.z = 0.3 * (0.4 - self.right_distance)
        elif self.turn_left and not rospy.is_shutdown():
            orientation_quaternion = msg.pose.pose.orientation
            orientation_euler = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])
            yaw_angle = orientation_euler[2]
            twist_cmd = Twist()
            angular_tolerance = 0.05 # Set a tolerance for stopping rotation
            if abs(self.target_yaw_angle_left - yaw_angle) > angular_tolerance:
                print("target yaw: ", self.target_yaw_angle_left)
                print("yaw: ", yaw_angle)
                # Calculate angular velocity based on the difference between current and target yaw angles
                twist_cmd.angular.z = 0.3 # * (self.target_yaw_angle_left - yaw_angle)
                self.pub.publish(twist_cmd)
            else:
                print("1 turn completed")
                self.pub.publish(Twist())  # Stop the robot
                self.turn_left = False
                
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
                self.pub.publish(Twist())  # Stop the robot
                self.turn_right = False

    def scan_callback(self, scan_msg):
        if not self.turn_left and not self.turn_right:
            self.right_distance = scan_msg.ranges[270]
            self.left_distance = scan_msg.ranges[90]
            back_distance = scan_msg.ranges[180]
            self.front_distance = scan_msg.ranges[0]

            # when a wall is detected in front and on the right
            if self.front_distance < 0.6 and self.right_distance < 1 and not rospy.is_shutdown():
                self.front_distance = scan_msg.ranges[0]
                self.twist.linear.x = 0.0
                self.move_forward = False
                self.turn_left = True
                print(self.turn_left)
                return  # Add a break statement to exit the loop
            
            # when a wall is detected in front and on the right
            elif self.front_distance < 0.6 and self.right_distance > 1 and not rospy.is_shutdown():
                print("No wall on right")
                self.front_distance = scan_msg.ranges[0]
                self.twist.linear.x = 0.0
                self.move_forward = False
                self.turn_right = True
                return
            
            elif self.front_distance > 1 and self.right_distance > 1 and self.left_distance < 0.6 and not rospy.is_shutdown():
                self.front_distance = scan_msg.ranges[0]

                # Set the duration to move forward in seconds
                move_forward_duration = rospy.Duration(5.0)  # Adjust the duration as needed

                # Get the current time
                start_time = rospy.Time.now()

                while not rospy.is_shutdown():
                    # Calculate the elapsed time
                    elapsed_time = rospy.Time.now() - start_time

                    if elapsed_time < move_forward_duration:
                        # Move forward
                        self.twist.linear.x = 0.2  # Adjust the linear velocity as needed
                    else:
                        # Stop moving forward
                        self.twist.linear.x = 0.0
                        self.move_forward = False
                        self.turn_right = True
                        break  # Exit the loop when the desired duration is reached

                    # Sleep for a short duration to control the loop rate
                    rospy.sleep(0.1)
    
                return

            # when a wall to the right and no wall in front
            elif self.front_distance > 0.4 and not rospy.is_shutdown():
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
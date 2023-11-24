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
        self.move_forward = False
        self.target_yaw_angle = None

    def odom_callback(self, msg):
        if not self.turn_left:
            orientation_quaternion = msg.pose.pose.orientation
            orientation_euler = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])
            yaw_angle = orientation_euler[2]
            self.target_yaw_angle = (yaw_angle + 1.5708) # 1.5708 radians is approximately 90 degrees

        if self.move_forward:
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.3 * (0.4 - self.right_distance)
        elif self.turn_left and not rospy.is_shutdown():
            orientation_quaternion = msg.pose.pose.orientation
            orientation_euler = euler_from_quaternion([orientation_quaternion.x, orientation_quaternion.y, orientation_quaternion.z, orientation_quaternion.w])
            yaw_angle = orientation_euler[2]
            twist_cmd = Twist()
            angular_tolerance = 0.05 # Set a tolerance for stopping rotation
            if abs(self.target_yaw_angle - yaw_angle) > angular_tolerance:
                print("target yaw: ", self.target_yaw_angle)
                print("yaw: ", yaw_angle)
                # Calculate angular velocity based on the difference between current and target yaw angles
                twist_cmd.angular.z = 0.3 * (self.target_yaw_angle - yaw_angle)
                self.pub.publish(twist_cmd)
            else:
                print("1 turn completed")
                self.pub.publish(Twist())  # Stop the robot
                self.turn_left = False
                




    def scan_callback(self, scan_msg):
        if not self.turn_left:
            self.right_distance = scan_msg.ranges[270]
            left_distance = scan_msg.ranges[90]
            back_distance = scan_msg.ranges[180]
            self.front_distance = scan_msg.ranges[0]

            # when a wall is detected in front
            if self.front_distance < 0.6 and not rospy.is_shutdown():
                self.front_distance = scan_msg.ranges[0]
                self.twist.linear.x = 0.0
                self.move_forward = False
                self.turn_left = True
                print(self.turn_left)
                return  # Add a break statement to exit the loop

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
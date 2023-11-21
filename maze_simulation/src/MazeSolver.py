#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry


class MazeSolver:
    def __init__(self):
        rospy.init_node('wall_follower', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.sub = rospy.Subscriber('/odom' , Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.twist = Twist()
        self.odom = Odometry()
    def odom_callback(self,msg):
        self.odom = msg
    def scan_callback(self, scan_msg):
        # Check if there is a wall in front
        right_distance = scan_msg.ranges[270]
        left_distance = scan_msg.ranges[90]
        back_distance = scan_msg.ranges[180]
        front_distance = scan_msg.ranges[0]

        # when wall to right and no wall in front    
        if front_distance > 0.5 or back_distance > 0.5:
            # move forward
            self.twist.linear.x = 0.2
    
        else:
            print(front_distance)
            print(back_distance)
            print("wall ahead")
            self.twist.linear.x = 0.0
            orientation_quaternion = self.odom.pose.pose.orientation
            orientation_euler = euler_from_quaternion([orientation_quaternion.x,orientation_quaternion.y,orientation_quaternion.z,orientation_quaternion.w])
            yaw_angle=orientation_euler[2]

            # Rotate the robot by 90 degrees
            target_yaw_angle = yaw_angle + 1.5708  # 1.5708 radians is approximately 90 degrees
            twist_cmd = Twist()
            while abs(yaw_angle - target_yaw_angle) > 0.05 :
                orientation_quaternion = self.odom.pose.pose.orientation
                orientation_euler = euler_from_quaternion([orientation_quaternion.x,orientation_quaternion.y,orientation_quaternion.z,orientation_quaternion.w])
                yaw_angle=orientation_euler[2]
                # Check if the robot has reached the target orientation
                    # Rotate the robot by publishing Twist messages
                twist_cmd.angular.z = 0.2  # Adjust the angular velocity as needed
                self.pub.publish(twist_cmd)

            print("out of while")
            self.pub.publish(Twist())  # Stop the robot
    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        mazeSolver = MazeSolver()
        mazeSolver.run()
    except rospy.ROSInterruptException:
        pass

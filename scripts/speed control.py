#!/usr/bin/env python3
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class SpeedControllerNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('speed_controller')

        # Define node parameters
        self.rate = rospy.get_param('~rate', 50.0)
        self.vehicle_length = rospy.get_param('~vehicle_length', 1.0)
        self.max_speed = rospy.get_param('~max_speed', 10.0)
        self.Kp = rospy.get_param('~Kp', 0.1)
        self.Ki = rospy.get_param('~Ki', 0.0)
        self.Kd = rospy.get_param('~Kd', 0.0)

        # Initialize PID controller variables
        self.prev_error = 0.0
        self.integral_error = 0.0

        # Initialize ROS publishers and subscribers
        self.drive_pub = rospy.Publisher('/ackermann_vehicle/ackermann_cmd', AckermannDriveStamped, queue_size=1)
        self.odom_sub = rospy.Subscriber('/ackermann_vehicle/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        # Get current speed and desired speed
        current_speed = msg.twist.twist.linear.x
        desired_speed = self.max_speed  # Replace with desired speed value from planner or joystick input

        # Calculate speed error
        error = desired_speed - current_speed

        # Calculate PID control action
        self.integral_error += error / self.rate
        derivative_error = (error - self.prev_error) * self.rate
        control_action = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error

        # Limit control action to achievable range
        if control_action > 1.0:
            control_action = 1.0
        elif control_action < -1.0:
            control_action = -1.0

        # Publish control action
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = control_action * self.max_speed
        drive_msg.drive.steering_angle = 0.0  # Replace with desired steering angle value from planner or joystick input
        self.drive_pub.publish(drive_msg)

        # Store previous error for next iteration
        self.prev_error = error

    def run(self):
        # Set ROS node rate
        rate = rospy.Rate(self.rate)

        # Run ROS node
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    node = SpeedControllerNode()
    node.run()

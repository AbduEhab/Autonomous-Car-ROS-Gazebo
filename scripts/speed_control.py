#!/usr/bin/env python3
import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import numpy as np
class SpeedControllerNode:
    def __init__(self):
        
        # Initialize ROS node
        rospy.init_node('speed_controller')
        self.last_log_time = rospy.Time.now()
        # Define node parameters
        self.rate = rospy.get_param('~rate', 30.0)
        self.vehicle_length = rospy.get_param('~vehicle_length', 1.0)
        self.prev_position = None
        self.max_speed = rospy.get_param('~max_speed', 10.0)
        self.logging_rate = rospy.get_param('~logging_rate', 10.0)
        self.Kp = rospy.get_param('~Kp', 0.1)
        self.Ki = rospy.get_param('~Ki', 0.0)
        self.Kd = rospy.get_param('~Kd', 0.0)
        self.current_time = rospy.Time.now()


        # Initialize PID controller variables
        self.prev_error = 0.0
        self.integral_error = 0.0

        # Initialize ROS publishers and subscribers
        self.drive_pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
        self.model_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)
        
        

    
    def model_callback(self, msg):
        delta_time = (rospy.Time.now() - self.current_time).to_sec()
        self.current_time = rospy.Time.now()
        rate = 1.0 / (delta_time+(1e-6))
        
        if len(msg.pose) < 2:
            return

        # Get current speed and desired speed
        current_speed = np.linalg.norm([msg.twist[1].linear.x, msg.twist[1].linear.y])

        desired_speed = self.max_speed

        # Calculate speed error
        error = desired_speed - current_speed
        error/=self.max_speed

        # Calculate PID control action
        self.integral_error += error / rate
        derivative_error = (error - self.prev_error) * rate
        control_action = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error

        # Limit control action to achievable range
        if control_action > 1.0:
            control_action = 1.0
        elif control_action < -1.0:
            control_action = -1.0  
        

        # Publish control action
        drive_msg = AckermannDrive()
        
        # drive_msg.speed = control_action * self.max_speed

        drive_msg.speed = self.max_speed
        drive_msg.steering_angle = 0.0
        if rospy.Time.now() - self.last_log_time > rospy.Duration(1.0 / self.logging_rate):
            self.last_log_time = rospy.Time.now()
            rospy.loginfo("\n Speed:, desired_speed: , error: ,   %f %f %f ", current_speed, desired_speed, control_action)
        #rospy.loginfo("%f, %f", msg.pose[1].position.x, msg.pose[1].position.y)
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

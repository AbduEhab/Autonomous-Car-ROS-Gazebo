#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

def __init__(self):
    # Initialize ROS node
    rospy.init_node('lateral_control')

    # Define node parameters
    self.rate = rospy.get_param('~rate', 30.0)
    self.vehicle_length = rospy.get_param('~vehicle_length', 1.0)
    self.max_speed = rospy.get_param('~max_speed', 10.0)
    self.logging_rate = rospy.get_param('~logging_rate', 10.0)
    self.current_time = rospy.Time.now()

# Define the Pure Pursuit controller function
def pure_pursuit_control(speed, heading, lateral_distance, goal_point):
    # Set the lookahead distance (L) and the maximum steering angle (max_steering_angle)
    L = 2.0
    max_steering_angle = math.pi / 4.0

    # Calculate the distance and angle between the vehicle and the goal point
    dx = goal_point.x - lateral_distance
    dy = goal_point.y
    distance = math.sqrt(dx*dx + dy*dy)
    angle = math.atan2(dy, dx)

    # Calculate the steering angle using the Pure Pursuit controller formula
    steering_angle = math.atan2(2.0 * L * math.sin(angle), distance)

    # Limit the steering angle to the maximum steering angle
    steering_angle = max(-max_steering_angle, min(steering_angle, max_steering_angle))

    # Calculate the control effort as a function of the steering angle and speed
    control_effort = speed / math.cos(steering_angle)

    return control_effort, steering_angle

def model_states_callback(msg):
    # Find the index of your vehicle model in the message
    # Replace <your_model_name> with the name of your vehicle model
    model_index = msg.name.index("<ackermann_vehicle>")

    # Extract the current position and orientation of your vehicle model
    x = msg.pose[model_index].position.x
    y = msg.pose[model_index].position.y
    z = msg.pose[model_index].position.z
    roll, pitch, yaw = euler_from_quaternion((msg.pose[model_index].orientation.x, msg.pose[model_index].orientation.y, msg.pose[model_index].orientation.z, msg.pose[model_index].orientation.w))

    # Extract the current linear velocity of your vehicle model
    current_speed = math.sqrt(msg.twist[model_index].linear.x ** 2 + msg.twist[model_index].linear.y ** 2 + msg.twist[model_index].linear.z ** 2)

    # Extract the lateral distance of your vehicle model
    lateral_distance = y

    # Publish the control effort and steering angle as a twist message
    control_msg = AckermannDrive()
    control_effort, steering_angle = pure_pursuit_control(current_speed, yaw, lateral_distance, goal_point)
    
    control_msg.steering_angle = steering_angle
    
    if rospy.Time.now() - last_log_time > rospy.Duration(1.0 / 10):
            last_log_time = rospy.Time.now()
            rospy.loginfo("Current speed: %f m/s, Current steering angle: %f degrees , control effort = ", current_speed, steering_angle * 180 / math.pi, control_effort)

    # Publish the control message on the control topic
    # Replace <your_control_topic> with the name of your control topic
    pub = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
    pub.publish(control_msg)

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('pure_pursuit_controller', anonymous=True)

    # Define the goal point as a Point message
    goal_point = Point()
    goal_point.x = 10.0
    goal_point.y = 0.0

    sub=rospy.Subscriber("/gazebo/model_states", ModelStates, model_states_callback)
    pub=rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)

    # Spin the node to receive callbacks and publish messages
    rospy.spin()

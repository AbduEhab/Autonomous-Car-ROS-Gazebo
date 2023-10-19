#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import json
class WayPoint:
    def __init__(self, x, y, start_distance, multiplier, index):
        self.multiplier = multiplier
        self.x = x * multiplier
        self.y = y * multiplier
        self.start_distance = start_distance
        self.visited = False
        self.index = index

    def get_in_local_frame(self,vehicle_x,vehicle_y,vehicle_heading):
        x = self.x - vehicle_x
        y = self.y - vehicle_y
        return x * math.cos(vehicle_heading) + y * math.sin(vehicle_heading), -x * math.sin(vehicle_heading) + y * math.cos(vehicle_heading)
    

class WayPointCollection:
    def __init__(self, waypoints):
        self.waypoints = waypoints
        self.start_distance = 0
        for i in range(1, len(self.waypoints)):
            self.waypoints[i].start_distance = self.waypoints[i - 1].start_distance + math.sqrt((self.waypoints[i].x - self.waypoints[i - 1].x) ** 2 + (self.waypoints[i].y - self.waypoints[i - 1].y) ** 2)
    
    def get_closest_waypoint(self, x, y):
        min_distance = float("inf") 
        nearest_waypoint = None
        for waypoint in self.waypoints:
            distance = math.sqrt((waypoint.x - x) ** 2 + (waypoint.y - y) ** 2)
            if distance < min_distance :
                min_distance = distance
                nearest_waypoint = waypoint
        return nearest_waypoint
    
    def get_lookahead_waypoint(self, x, y, lookahead_distance):
        closest_waypoint = self.get_closest_waypoint(x, y)
        lookahead_distance += closest_waypoint.start_distance
        for waypoint in self.waypoints:
            if waypoint.start_distance > lookahead_distance:
                waypoint.visited = True
                return waypoint
        waypoint.visited = True
        return self.waypoints[-1]

class LateralControlNode:

    def __init__(self):
        # Initialize ROS node
        
        rospy.init_node('lateral_control')
        self.last_log_time = rospy.Time.now()
        self.waypoint_paths = rospy.get_param("~waypoints_path", "")
        self.initialize_waypoints()
        # Define node parameters
        self.rate = rospy.get_param('~rate', 30.0)
        self.vehicle_length = rospy.get_param('~vehicle_length', 1.0)
        self.max_speed = rospy.get_param('~max_speed', 10.0)
        self.logging_rate = rospy.get_param('~logging_rate', 10.0)
        self.vehicle_path_file = rospy.get_param('~vehicle_path_file', "")
        self.current_time = rospy.Time.now()
        self.sub=rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        self.pub=rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
        self.waypoints_collection = WayPointCollection([WayPoint(waypoint[0], waypoint[1], 0, 20,i) for i,waypoint in enumerate(self.waypoints)])
        self.visited_waypoints = []
        self.vehicle_path = []


    def initialize_waypoints(self):
        if self.waypoint_paths == "":
            raise FileNotFoundError("Couldn't find waypoints file")
        with open(self.waypoint_paths, "r") as f:
            self.waypoints = json.load(f)["waypoints"]

    def pure_pursuit(self, x, y, yaw, waypoints, lookahead_distance):
        
        # Find the lookahead waypoint
        lookahead_waypoint = waypoints.get_lookahead_waypoint(x, y, lookahead_distance)
        
        # Find the distance between the vehicle and the lookahead waypoint
        x, y = lookahead_waypoint.get_in_local_frame(x, y, yaw)
        lookahead_distance = math.sqrt(x ** 2 + y ** 2)
        
        # Find the angle between the vehicle heading and the lookahead waypoint
        lookahead_angle = math.atan2(y, x)
        lookahead_angle = min(lookahead_angle, math.pi / 4)
        lookahead_angle = max(lookahead_angle, -math.pi / 4)
        
        return lookahead_angle, lookahead_distance ,lookahead_waypoint
    
    def export_vehicle_path(self):
        with open(self.vehicle_path_file, "w+") as f:
            json.dump({"vehicle_path": self.vehicle_path}, f)

    def model_states_callback(self,msg):
        # Find the index of your vehicle model in the message
        
        
        if len(msg.pose) < 2:
            return

        # model_index = msg.name.index("<ackermann_vehicle>")
        model_index = 1
       
        # print(waypoints)
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
        self.vehicle_path.append((x, y))
        control_msg = AckermannDrive()
        look_ahead_angle, look_ahead_distance, look_ahead_waypoint = self.pure_pursuit(x, y, yaw, self.waypoints_collection, 2)
        closest_waypoint = self.waypoints_collection.get_closest_waypoint(x, y)
        if len(self.visited_waypoints) == 0 or self.visited_waypoints[-1] != closest_waypoint.index:
            self.visited_waypoints.append(closest_waypoint.index)
        if len(self.visited_waypoints)>len(self.waypoints):
            self.export_vehicle_path()
            rospy.signal_shutdown("Reached goal")
        
        steering_angle = look_ahead_angle
        control_msg.steering_angle = steering_angle
        control_msg.steering_angle_velocity = 0
        control_msg.speed = current_speed
        if rospy.Time.now() - self.last_log_time > rospy.Duration(1.0 / self.logging_rate):
            self.last_log_time = rospy.Time.now()
            rospy.loginfo("%s", str(self.visited_waypoints))
    
        # Publish the control message on the control topic
        # Replace <your_control_topic> with the name of your control topic
        self.pub.publish(control_msg)

    def run(self):
        # Set ROS node rate
        
        rate = rospy.Rate(self.rate)
        

        # Run ROS node
        
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':

    node=LateralControlNode()
    node.run()

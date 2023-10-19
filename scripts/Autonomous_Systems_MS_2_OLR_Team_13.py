#!/usr/bin/env python3


import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates  # For ModelStates
import tf


# rostopic pub -r 10 /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.2}}'


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)

# it gives me the wrong parameteres
# def handle_vehicle_pose(state: ModelStates, vehicle_name):
#     try:
#       vehicle_index = state.name.index(vehicle_name)
#     except:
#       return

#     pose, twist = state.pose[vehicle_index], state.twist[vehicle_index]
#     vel = twist.linear.x
#     steer = math.degrees(twist.angular.z)
#     pos = pose.position.x
#     theta = math.degrees(pose.orientation.z)
#     rospy.loginfo(f"\ntwist_vel  {vel:.2f} twist_steer  {steer:.1f} position  {pos:.2f} theta  {theta:.1f}")1


def handle_vehicle_pose(state: ModelStates):
    global vehicle_name
    global last_log_time
    try:
        vehicle_index = state.name.index(vehicle_name)
    except ValueError:
        return
    
    pose, twist = state.pose[vehicle_index], state.twist[vehicle_index]
    vel = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
    steer = twist.angular.z
    pos_x = pose.position.x
    pos_y = pose.position.y
    pos_z = pose.position.z
    theta = tf.transformations.euler_from_quaternion(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]  # yaw
    if rospy.Time.now() - last_log_time > rospy.Duration(0.087):
        last_log_time = rospy.Time.now()
        rospy.loginfo(
            f"\ntwist_vel:  {vel:.2f} twist_steer_velocity:  {steer:.2f} position in x:  {pos_x:.2f} Position in y :{pos_y:.2f} Position in z:{pos_z:.2f}orientation  {theta:.2f}")


def publish_message(event):
    global wheelbase  # wheelbase is the distance between the front and rear wheels
    # ackermann_cmd_topic is the topic to which the AckermannDrive message is published
    global ackermann_cmd_topic
    global frame_id  # frame_id is the frame of the AckermannDrive message
    global pub  # pub is the publisher of the AckermannDrive message
    global message_type  # message_type is the type of the AckermannDrive message

    if message_type == 'ackermann_drive':
        v = float(rospy.get_param('~Velocity'))
        steering = float(rospy.get_param('~Steer_Velocity'))

        steering = convert_trans_rot_vel_to_steering_angle(
            v, steering, wheelbase)

        msg = AckermannDrive()
        msg.steering_angle = steering
        msg.speed = v

        pub.publish(msg)

    else:
        v = float(rospy.get_param('~Velocity'))
        steering = float(rospy.get_param('~Steer_Velocity'))
        steering = convert_trans_rot_vel_to_steering_angle(
            v, steering, wheelbase)

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.drive.steering_angle = np.pi/4
        msg.drive.speed = v

        pub.publish(msg)


if __name__ == '__main__':
    try:

        
        rospy.init_node(
            'cmd_vel_to_ackermann_drive_Autonomous_Systems_MS_2_OLR_Team_13')
        vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
        time_step = rospy.get_param('~time_step')
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
        ackermann_cmd_topic = rospy.get_param(
            '~ackermann_cmd_topic', '/ackermann_cmd')
        wheelbase = rospy.get_param('~wheelbase', 1.0)
        frame_id = rospy.get_param('~frame_id', 'odom')
        # ackermann_drive or ackermann_drive_stamped
        message_type = rospy.get_param('~message_type', 'ackermann_drive')
        last_log_time = rospy.Time.now()
        #  rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)
        if message_type == 'ackermann_drive':
            pub = rospy.Publisher(ackermann_cmd_topic,
                                  AckermannDrive, queue_size=1)
        else:
            pub = rospy.Publisher(ackermann_cmd_topic,
                                  AckermannDriveStamped, queue_size=1)
        # rospy.Subscriber('/gazebo/model_states',
        #                  ModelStates,
        #                  handle_vehicle_pose, vehicle_name
        #                  )
        rospy.Subscriber('/gazebo/model_states',
                         ModelStates,
                         handle_vehicle_pose
                         )
        rospy.loginfo("Node 'Autonomous_Systems_MS_2_OLR_Team_13' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f",
                      "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)
        # Create a timer that calls the publish_message function every 1 second
        rospy.Timer(rospy.Duration(time_step), publish_message)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

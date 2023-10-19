#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from ackermann_msgs.msg import AckermannDrive 
from gazebo_msgs.msg import ModelStates  # For ModelStates
import tf
import curses
# Initialize curses
stdscr = curses.initscr()
curses.cbreak()
stdscr.keypad(True)

# Initialize velocities and steering angle
linear_vel = 0.0
steering_vel = 0.0
linear_acc  = 0.0
def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    if omega == 0 or v == 0:
        return 0

    radius = v / omega
    return math.atan(wheelbase / radius)


def handle_vehicle_pose(state: ModelStates):
    global vehicle_name
    try:
        vehicle_index = state.name.index(vehicle_name)
    except ValueError:
        return
    
    pose, twist = state.pose[vehicle_index], state.twist[vehicle_index]
    vel = twist.linear.x
    
    steer = twist.angular.z
    pos = pose.position.x
    theta = tf.transformations.euler_from_quaternion(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]

    # rospy.loginfo(
    #     f"\ntwist_vel  {vel:.2f} twist_steer  {steer:.2f} position  {pos:.2f} theta  {theta:.2f}")

def get_arrow_key(key):
    # Convert arrow key input to linear and steering velocity
    if key == curses.KEY_UP:
        return 1, 0
    elif key == curses.KEY_DOWN:
        return -1, 0
    elif key == curses.KEY_LEFT:
        return 0, 1
    elif key == curses.KEY_RIGHT:
        return 0, -1
    else:
        return 0, 0
def publish_message(event):
    global wheelbase  # wheelbase is the distance between the front and rear wheels
    # ackermann_cmd_topic is the topic to which the AckermannDrive message is published
    global ackermann_cmd_topic
    global frame_id  # frame_id is the frame of the AckermannDrive message
    global pub  # pub is the publisher of the AckermannDrive message
    global message_type  # message_type is the type of the AckermannDrive message

     # Update velocities based on key presses
    global linear_vel
    global steering_vel
    global linear_acc
    
    v = linear_vel
    a = linear_acc
    steering = steering_vel
    if stdscr.getch() == curses.KEY_UP:
        linear_vel += 0.1
        linear_acc += 0.1
    elif stdscr.getch() == curses.KEY_DOWN:
        linear_vel -= 0.1
        linear_acc -= 0.1
    elif stdscr.getch() == curses.KEY_LEFT:
        steering_vel += 0.1 
    elif stdscr.getch() == curses.KEY_RIGHT:
        steering_vel -= 0.1
    # elif stdscr.getch() == curses.keyname('q'):
    #     linear_vel = 0.0
    #     steering_vel = 0.0
    #     rospy.signal_shutdown("Quit")

    # calculate steering angle based on velocity and wheelbase
    steering_angle = convert_trans_rot_vel_to_steering_angle(
        v, steering, wheelbase)

    # publish message
    if message_type == 'ackermann_drive':
        msg = AckermannDrive()
        msg.steering_angle = steering_angle
        msg.speed = linear_vel
        msg.acceleration = linear_acc
        msg.steering_angle_velocity = steering_vel
        # rospy.loginfo(
        #     f"linear velocity: {v:.2f} m/s, steering angle: {steering_angle:.2f} rad")

        pub.publish(msg)

    else:
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = linear_vel
        msg.drive.acceleration = linear_acc
        msg.drive.steering_angle_velocity = steering_vel

        pub.publish(msg)

    # log velocity and steering angle
    rospy.loginfo( f"linear velocity: {v:.2f} m/s, linear acceleration: {a:.2f}, steering angle: {steering_angle:.2f} rad")


if __name__ == '__main__':
    try:

        rospy.init_node(
            'cmd_vel_to_ackermann_drive_Autonomous_Systems_MS_2_OLR_Team_13')
        vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
        #time_step = rospy.get_param('~time_step')
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
        ackermann_cmd_topic = rospy.get_param(
            '~ackermann_cmd_topic', '/ackermann_cmd')
        wheelbase = rospy.get_param('~wheelbase', 1.0)
        frame_id = rospy.get_param('~frame_id', 'odom')
        # ackermann_drive or ackermann_drive_stamped
        message_type = rospy.get_param('~message_type', 'ackermann_drive')

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
        rospy.Timer(rospy.Duration(0.01), publish_message)
        rospy.spin()
        # Clean up curses
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()


    except rospy.ROSInterruptException:
        pass

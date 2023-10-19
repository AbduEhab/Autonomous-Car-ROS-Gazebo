#!/usr/bin/env python3
import rospy # get the python library for ROS

rospy.init_node('Validation_Printing_Node_Team_13') # initialize the node
for i in range(0, 10):
    rospy.loginfo('Hello World!!!!!!')
    rospy.sleep(1)
print('bye World!!!!!!')

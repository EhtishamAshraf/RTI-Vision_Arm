#!/usr/bin/env python3

"""
This file is only used for simulating the behaviour of a "xy plotter"

Commands to control without this .py file:
rostopic pub /xy_plotter/base_to_slider_joint_position_controller/command std_msgs/Float64 0.25
rostopic pub /xy_plotter/slider_to_second_joint_position_controller/command std_msgs/Float64 0.1


when using the Robotic arm and the xy platform together, I have put the controllers of the xy platform 
inside the namespace /eva so I have replaced /xy_plotter with /eva in the Publisher:
-       /eva/base_to_slider_joint_position_controller/command
-       /eva/slider_to_second_joint_position_controller/command
"""

import rospy
from std_msgs.msg import Float64

def move_joints():
    # Initialize the ROS node
    rospy.init_node('xyPlotter_controller', anonymous=True)

    # Create publishers for the joint controllers:
    """use when using the xy_platform alone, inside the xy_platform_control package"""
    # x_joint_pub = rospy.Publisher('/xy_plotter/base_to_slider_joint_position_controller/command', Float64, queue_size=10)
    # y_joint_pub = rospy.Publisher('/xy_plotter/slider_to_second_joint_position_controller/command', Float64, queue_size=10)
    
    """use when using the xy_platform with eva arm, inside the eva_control_without_rail package"""
    x_joint_pub = rospy.Publisher('/eva/base_to_slider_joint_position_controller/command', Float64, queue_size=10)
    y_joint_pub = rospy.Publisher('/eva/slider_to_second_joint_position_controller/command', Float64, queue_size=10)

    rate = rospy.Rate(10)
    rospy.sleep(3)

    # Move the X-axis joint to -0.175
    x_joint_pub.publish(-0.175)
    rospy.loginfo("Moved X-axis joint to -0.175")

    rospy.sleep(2)

    # Move the Y-axis joint to -0.1
    y_joint_pub.publish(-0.1)
    rospy.loginfo("Moved Y-axis joint to -0.1")

    rospy.sleep(2)
    
    # Move the X-axis joint to 0.175 
    x_joint_pub.publish(0.175)
    rospy.loginfo("Moved X-axis joint to 0.175")

    rospy.sleep(2)

    # Move the Y-axis joint to 0.1
    y_joint_pub.publish(0.1)
    rospy.loginfo("Moved Y-axis joint to 0.1")

    rospy.sleep(2)

    # Move the X-axis joint back to 0.0
    x_joint_pub.publish(0.0)
    rospy.loginfo("Moved X-axis joint to 0.0")

    # Move the Y-axis joint back to 0.0
    y_joint_pub.publish(0.0)
    rospy.loginfo("Moved Y-axis joint to 0.0")

if __name__ == '__main__':
    try:
        move_joints()
    except rospy.ROSInterruptException:
        pass